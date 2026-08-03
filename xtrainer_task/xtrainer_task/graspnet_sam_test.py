#!/usr/bin/env python3
"""GroundedSAM2 (Grounding DINO + SAM2) + GraspNet 组合测试。

流程:
    1. 订阅 ``/camera/camera_top/color/image_raw`` (彩色图) 与
       ``/camera/camera_top/depth/color/points`` (有序彩色点云)。
    2. 用 Grounding DINO + SAM2 在彩色图上分割出 prompt 指定的物体 → 得到 2D mask。
    3. RealSense 的点云是 *有序* 的 (height×width 与彩色图对齐)，把 2D mask
       作用到点云的 H×W 网格上，取出目标物体的 3D 点子集。
    4. 在该子集上运行 GraspNet 生成抓取姿态。

显示:
    - OpenCV 窗口: SAM2 分割结果 (框 + mask)
    - Open3D 窗口: 目标物体点云 + GraspNet 夹爪 (流式刷新)

按键:
    OpenCV 窗口按 ``q`` 或 Open3D 窗口按 ``Esc``/``Q`` 退出。

Boot::

    export PYTHONPATH=/opt/Project/Grounded-SAM-2/grounding_dino:/opt/Project/Grounded-SAM-2:/home/kira/miniconda3/lib/python3.12/site-packages:$PYTHONPATH
    ros2 run xtrainer_task graspnet_sam_test --ros-args -p pointcloud_topic:=/camera/camera_right/depth/color/points -p image_topic:=/camera/camera_right/color/image_raw -p prompt:=bottle

.. note::

    不依赖 ``cv_bridge`` (与 conda numpy 2.x 冲突)，直接解析 ROS 原始消息。
"""

import os
import sys
import threading
import time
import traceback

import cv2
import numpy as np
import open3d as o3d
import rclpy
import torch
from ament_index_python.packages import get_package_share_directory
from graspnetAPI import GraspGroup
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2

from graspnet.graspnet import GraspNet, pred_decode
from xtrainer_task.dino_test import ros_image_to_cv2
from xtrainer_task.dino_wrapper import DinoWrapper

# ── PointCloud2 datatype → numpy ──────────────────────────────
_DTYPE_MAP = {
    1: np.int8,
    2: np.int16,
    4: np.int32,
    5: np.uint16,
    6: np.uint32,
    7: np.float32,
    8: np.float64,
}


class GraspNetSamTestNode(Node):
    """ROS2 节点：GroundedSAM2 分割点云 + GraspNet 抓取检测，流式显示。"""

    def __init__(self):
        super().__init__("graspnet_sam_test")

        model_path = os.path.join(
            get_package_share_directory("graspnet"),
            "model",
            "checkpoint-rs.tar",
        )

        # ── 参数 ──────────────────────────────────────────────
        self.declare_parameter("image_topic", "/camera/camera_top/color/image_raw")
        self.declare_parameter("pointcloud_topic", "/camera/camera_top/depth/color/points")
        self.declare_parameter("prompt", "objects.")
        self.declare_parameter("box_threshold", 0.35)
        self.declare_parameter("text_threshold", 0.25)
        self.declare_parameter("max_distance", 1.5)
        self.declare_parameter("checkpoint_path", model_path)
        self.declare_parameter("num_point", 20000)
        self.declare_parameter("num_view", 300)
        self.declare_parameter("collision_thresh", 0.01)
        self.declare_parameter("voxel_size", 0.01)
        self.declare_parameter("top_k_grasps", 5)
        # 夹爪长度(depth)上限, 单位 m, 过滤 depth>此值的抓取。9.5cm=0.095
        self.declare_parameter("max_grasp_depth", 0.095)
        # mask 选择: -1=所有 mask 并集(默认), >=0=按面积降序的第 i 个
        self.declare_parameter("mask_index", -1)
        self.declare_parameter("min_infer_interval", 0.0)
        self.declare_parameter("show_2d", True)
        # SAM 边缘膨胀: 3D 空间把分割点云往外多保留 radius_m 内的点
        self.declare_parameter("mask_dilation_m", 0.01)

        self._image_topic = self.get_parameter("image_topic").value
        self._pointcloud_topic = self.get_parameter("pointcloud_topic").value
        self._max_distance = self.get_parameter("max_distance").value
        self._checkpoint_path = self.get_parameter("checkpoint_path").value
        self._num_point = self.get_parameter("num_point").value
        self._num_view = self.get_parameter("num_view").value
        self._collision_thresh = self.get_parameter("collision_thresh").value
        self._voxel_size = self.get_parameter("voxel_size").value
        self._top_k_grasps = self.get_parameter("top_k_grasps").value
        self._max_grasp_depth = self.get_parameter("max_grasp_depth").value
        self._mask_index = self.get_parameter("mask_index").value
        self._min_infer_interval = self.get_parameter("min_infer_interval").value
        self._show_2d = self.get_parameter("show_2d").value
        self._mask_dilation_m = self.get_parameter("mask_dilation_m").value

        # ── 加载 GroundingDINO + SAM2 ──────────────────────────
        self.get_logger().info("Loading Grounding DINO + SAM2 models …")
        self._dino = DinoWrapper(
            box_threshold=self.get_parameter("box_threshold").value,
            text_threshold=self.get_parameter("text_threshold").value,
        )
        self.get_logger().info("Detection models loaded.")

        # ── 加载 GraspNet ──────────────────────────────────────
        self._graspnet = None
        if self._checkpoint_path:
            self.get_logger().info(f"Loading GraspNet model from {self._checkpoint_path} …")
            self._graspnet = self._load_graspnet()
            self.get_logger().info("GraspNet model loaded successfully.")
        else:
            self.get_logger().fatal("No checkpoint_path provided.")
            rclpy.shutdown()
            return

        # ── 流式状态 ──────────────────────────────────────────
        self._latest_image = None
        self._latest_cloud = None
        self._buf_lock = threading.Lock()  # 保护 _latest_image/_latest_cloud

        self._result_lock = threading.Lock()
        self._result_cloud = None
        self._result_geometries = None

        self._running = True
        self._last_log = 0.0

        # ── Open3D 几何体 (主线程维护) ────────────────────────
        self._vis = None
        self._cloud_geo = o3d.geometry.PointCloud()
        self._gripper_geos = []
        self._geo_added = False

        # ── 订阅 ───────────────────────────────────────────────
        self.create_subscription(Image, self._image_topic, self._image_callback, 10)
        self.create_subscription(PointCloud2, self._pointcloud_topic, self._cloud_callback, 10)

        self.get_logger().info(
            f'Subscribed image="{self._image_topic}", cloud="{self._pointcloud_topic}". '
            f'Waiting for data ...'
        )

        # ── 启动推理线程 ──────────────────────────────────────
        self._worker = threading.Thread(target=self._inference_loop, daemon=True)
        self._worker.start()

    # ── 回调：只缓存最新帧 ─────────────────────────────────────
    def _image_callback(self, msg: Image):
        with self._buf_lock:
            self._latest_image = msg

    def _cloud_callback(self, msg: PointCloud2):
        with self._buf_lock:
            self._latest_cloud = msg

    # ── 后台推理线程 ──────────────────────────────────────────
    def _inference_loop(self):
        while self._running and rclpy.ok():
            # 取最新的一对 (image, cloud)
            with self._buf_lock:
                image_msg = self._latest_image
                cloud_msg = self._latest_cloud
                # 配对后清空 cloud, 避免重复处理; image 可保留(下次配新 cloud)
                if image_msg is not None and cloud_msg is not None:
                    self._latest_cloud = None

            if image_msg is None or cloud_msg is None:
                time.sleep(0.02)
                continue

            t0 = time.time()
            try:
                self._process_frame(image_msg, cloud_msg)
            except Exception:
                self.get_logger().error(f"Frame processing failed:\n{traceback.format_exc()}")

            dt = time.time() - t0
            if self._min_infer_interval > dt:
                time.sleep(self._min_infer_interval - dt)

    def _process_frame(self, image_msg: Image, cloud_msg: PointCloud2):
        """单帧处理：SAM2 分割 → mask 点云 → GraspNet。"""
        prompt = self.get_parameter("prompt").value

        # ── 1. ROS Image → BGR ────────────────────────────────
        frame = ros_image_to_cv2(image_msg)

        # ── 2. GroundingDINO + SAM2 ──────────────────────────
        det = self._dino.detect(frame, prompt)

        # ── 3. 解析有序点云 ───────────────────────────────────
        xyz, rgb, H, W = self._ros_pointcloud_to_organized(cloud_msg)

        # ── 4. 取 mask 并作用到点云网格 ───────────────────────
        mask2d = self._select_mask(det, H, W)
        if mask2d is not None:
            mask2d = self._dilate_mask_3d(mask2d, xyz, self._mask_dilation_m)

        grippers = []
        grasp_axes = []
        if mask2d is not None:
            valid = np.isfinite(xyz).all(axis=2)
            if self._max_distance > 0:
                valid &= (np.linalg.norm(xyz, axis=2) <= self._max_distance)
            sel = mask2d & valid
            pts = xyz[sel]
            cols = rgb[sel]
        else:
            pts = np.empty((0, 3), dtype=np.float32)
            cols = np.empty((0, 3), dtype=np.float32)

        cloud_o3d = None
        if len(pts) > 0:
            cloud_o3d = o3d.geometry.PointCloud()
            cloud_o3d.points = o3d.utility.Vector3dVector(pts)
            cloud_o3d.colors = o3d.utility.Vector3dVector(cols)

            # ── 5. GraspNet 抓取检测 ──────────────────────────
            gg = self._predict_grasps(cloud_o3d)
            if gg is not None and len(gg) > 0:
                grippers, grasp_axes = self._filter_and_vis_grasps(gg)

        # ── 6. 结果交给主线程显示 ─────────────────────────────
        with self._result_lock:
            self._result_cloud = cloud_o3d
            self._result_geometries = grippers + grasp_axes

        # ── 7. 2D 分割结果 OpenCV 显示 ───────────────────────
        if self._show_2d:
            annotated = DinoWrapper.annotate(frame, det)
            n_pts = len(pts)
            cv2.putText(
                annotated,
                f"pts={n_pts} grasps={len(grippers)}",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2,
            )
            cv2.imshow("GroundedSAM2 Result", annotated)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                self.get_logger().info("Pressed 'q' in OpenCV window, exiting …")
                self._running = False

        now = time.time()
        if now - self._last_log > 1.0:
            self._last_log = now
            self.get_logger().info(
                f"dets={len(det.boxes)} pts={len(pts)} grasps={len(grippers)} "
                f"({now - t0:.2f}s)"
            )

    # ── mask 选择 ─────────────────────────────────────────────
    def _select_mask(self, det, H_pc, W_pc):
        """从检测结果里取出一个 (H_pc, W_pc) 的 bool mask。无检测返回 None。"""
        if len(det.boxes) == 0:
            return None

        masks = det.masks  # (N, H_img, W_img) bool
        H_img, W_img = masks.shape[1], masks.shape[2]

        # 分辨率不一致时按最近邻缩放 mask 到点云尺寸
        if (H_img, W_img) != (H_pc, W_pc):
            resized = np.zeros((masks.shape[0], H_pc, W_pc), dtype=bool)
            for i in range(masks.shape[0]):
                resized[i] = cv2.resize(
                    masks[i].astype(np.uint8),
                    (W_pc, H_pc),
                    interpolation=cv2.INTER_NEAREST,
                ).astype(bool)
            masks = resized

        if self._mask_index < 0:
            # 所有 mask 并集
            return np.any(masks, axis=0)

        # 按面积降序排列后取第 mask_index 个
        areas = masks.reshape(masks.shape[0], -1).sum(axis=1)
        order = np.argsort(-areas)
        idx = int(self._mask_index)
        if idx >= len(order):
            idx = 0
        return masks[order[idx]]

    @staticmethod
    def _dilate_mask_3d(mask2d, xyz, radius_m: float):
        """3D 空间膨胀 mask: 保留 mask 点云周围 radius_m (米) 内的所有点云点。

        SAM 边缘常略紧 (少保留了一些物体表面点), 用该参数把分割点云
        往外多保留一圈。radius_m<=0 时直接返回原 mask。
        """
        if radius_m <= 0:
            return mask2d

        valid = np.isfinite(xyz).all(axis=2) & (xyz[..., 2] > 0)
        sel = mask2d & valid
        pts = xyz[sel].reshape(-1, 3)
        if len(pts) == 0:
            return mask2d

        from scipy.spatial import cKDTree
        tree = cKDTree(pts)

        flat_valid = valid.reshape(-1)
        idx = np.where(flat_valid)[0]
        if len(idx) == 0:
            return mask2d

        grid = xyz.reshape(-1, 3)
        dists, _ = tree.query(grid[idx], k=1)

        dilated = mask2d.copy()
        dilated.reshape(-1)[idx[dists <= radius_m]] = True
        return dilated

    # ── 有序点云解析 ──────────────────────────────────────────
    @staticmethod
    def _ros_pointcloud_to_organized(msg: PointCloud2):
        """解析有序 PointCloud2 → (xyz(H,W,3), rgb(H,W,3), H, W)。"""
        field_names = [f.name for f in msg.fields]
        if not all(k in field_names for k in ("x", "y", "z")):
            raise ValueError(f"PointCloud2 missing x/y/z fields, got {field_names}")

        H, W = msg.height, msg.width
        if H <= 1:
            raise ValueError(
                f"Point cloud is not organized (height={H}); "
                "cannot apply 2D segmentation mask."
            )

        point_step = msg.point_step
        n = H * W
        raw = np.frombuffer(msg.data, dtype=np.uint8)[: n * point_step].reshape(n, point_step)

        offsets = {f.name: (f.offset, _DTYPE_MAP.get(f.datatype, np.float32)) for f in msg.fields}

        def get_field(name):
            off, dt = offsets[name]
            sz = np.dtype(dt).itemsize
            return np.frombuffer(raw[:, off: off + sz].tobytes(), dtype=dt).astype(np.float32)

        x = get_field("x").reshape(H, W)
        y = get_field("y").reshape(H, W)
        z = get_field("z").reshape(H, W)
        xyz = np.stack([x, y, z], axis=2)  # (H, W, 3)

        rgb = np.zeros((H, W, 3), dtype=np.float32)
        rgb_field = "rgb" if "rgb" in field_names else ("rgba" if "rgba" in field_names else None)
        if rgb_field is not None:
            off, dt = offsets[rgb_field]
            sz = np.dtype(dt).itemsize
            rgb_raw = raw[:, off: off + sz].tobytes()
            if dt == np.float32:
                packed = np.frombuffer(rgb_raw, dtype=np.float32).view(np.uint32)
            else:
                packed = np.frombuffer(rgb_raw, dtype=dt).astype(np.uint32)

            r = ((packed >> 16) & 0xFF).astype(np.float32) / 255.0
            g = ((packed >> 8) & 0xFF).astype(np.float32) / 255.0
            b = (packed & 0xFF).astype(np.float32) / 255.0
            rgb = np.stack([r, g, b], axis=1).reshape(H, W, 3)

        return xyz, rgb, H, W

    # ── GraspNet 相关 (与 graspnet_test 一致) ─────────────────
    def _load_graspnet(self) -> GraspNet:
        net = GraspNet(
            input_feature_dim=0,
            num_view=self._num_view,
            num_angle=12,
            num_depth=4,
            cylinder_radius=0.05,
            hmin=-0.02,
            hmax_list=[0.01, 0.02, 0.03, 0.04],
            is_training=False,
        )
        device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
        net.to(device)
        checkpoint = torch.load(self._checkpoint_path, map_location=device)
        net.load_state_dict(checkpoint["model_state_dict"])
        start_epoch = checkpoint.get("epoch", "?")
        self.get_logger().info(f"Loaded checkpoint (epoch: {start_epoch})")
        net.eval()
        return net

    def _predict_grasps(self, cloud_o3d: o3d.geometry.PointCloud):
        points = np.asarray(cloud_o3d.points, dtype=np.float32)
        colors = np.asarray(cloud_o3d.colors, dtype=np.float32)
        if len(points) == 0:
            return None

        # ── 降采样到 num_point ───────────────────────────────
        if len(points) >= self._num_point:
            idxs = np.random.choice(len(points), self._num_point, replace=False)
        else:
            idxs1 = np.arange(len(points))
            idxs2 = np.random.choice(len(points), self._num_point - len(points), replace=True)
            idxs = np.concatenate([idxs1, idxs2], axis=0)

        cloud_sampled = points[idxs]
        color_sampled = colors[idxs] if len(colors) == len(points) else np.zeros_like(points)

        device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
        cloud_t = torch.from_numpy(cloud_sampled[np.newaxis].astype(np.float32)).to(device)

        end_points = {
            "point_clouds": cloud_t,
            "cloud_colors": color_sampled,
        }

        with torch.no_grad():
            end_points = self._graspnet(end_points)
            grasp_preds = pred_decode(end_points)

        gg_array = grasp_preds[0].detach().cpu().numpy()
        return GraspGroup(gg_array)

    def _filter_and_vis_grasps(self, gg):
        """过滤抓取并生成可视化几何体 (夹爪图标 + 每抓取坐标系)。

        Parameters
        ----------
        gg : GraspGroup
            原始抓取集合

        Returns
        -------
        tuple[list, list]
            (夹爪三角网格列表, 每抓取坐标系列表, 相机系, 调试方向用)
        """
        # 过滤夹爪长度(depth) > max_grasp_depth 的抓取
        if self._max_grasp_depth > 0 and len(gg) > 0:
            keep = np.where(gg.depths <= self._max_grasp_depth)[0]
            gg = gg[keep]
        gg.nms()
        gg.sort_by_score()
        gg = gg[: self._top_k_grasps]

        geoms = gg.to_open3d_geometry_list()
        axes = []
        for g in gg:
            T = np.eye(4)
            T[:3, :3] = g.rotation_matrix
            T[:3, 3] = g.translation
            axes.append(
                o3d.geometry.TriangleMesh.create_coordinate_frame(
                    size=0.03
                ).transform(T)
            )
        return geoms, axes

    # ── 主线程：刷新显示 ──────────────────────────────────────
    def attach_visualizer(self, vis):
        self._vis = vis

    def update_frame(self) -> bool:
        """消费最新推理结果并刷新 Open3D 窗口。返回 False 表示窗口已关闭。"""
        if self._vis is None:
            return True

        have_update = False
        with self._result_lock:
            if self._result_cloud is not None:
                cloud = self._result_cloud
                grippers = self._result_geometries or []
                self._result_cloud = None
                self._result_geometries = None
                have_update = True

        if have_update:
            self._cloud_geo.points = cloud.points
            self._cloud_geo.colors = cloud.colors
            if not self._geo_added:
                self._vis.add_geometry(self._cloud_geo, reset_bounding_box=True)
                self._geo_added = True
            else:
                self._vis.update_geometry(self._cloud_geo)

            for g in self._gripper_geos:
                self._vis.remove_geometry(g, reset_bounding_box=False)
            self._gripper_geos = list(grippers)
            for g in self._gripper_geos:
                self._vis.add_geometry(g, reset_bounding_box=False)

        ok = self._vis.poll_events()
        self._vis.update_renderer()
        return ok


def main():
    rclpy.init(args=sys.argv)

    node = GraspNetSamTestNode()
    if not node._graspnet:
        return

    # ── Open3D 持久窗口 ────────────────────────────────────
    vis = o3d.visualization.VisualizerWithKeyCallback()
    vis.create_window(
        window_name="GraspNet + GroundedSAM2 - Segmented Point Cloud",
        width=1280,
        height=720,
    )
    node.attach_visualizer(vis)
    # 参考坐标轴 (相机光学系): X红 Y绿 Z蓝
    vis.add_geometry(
        o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)
    )

    def _on_quit(_vis):
        node.get_logger().info("Quit requested.")
        node._running = False
        return False

    vis.register_key_callback(27, _on_quit)          # Esc
    vis.register_key_callback(ord("Q"), _on_quit)    # Q

    # ── 主循环 ──────────────────────────────────────────────
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)

    try:
        while rclpy.ok() and node._running:
            executor.spin_once(timeout_sec=0.01)
            if not node.update_frame():
                node.get_logger().info("Visualization window closed, shutting down …")
                break
    except KeyboardInterrupt:
        pass
    finally:
        node._running = False
        if node._worker.is_alive():
            node._worker.join(timeout=2.0)
        cv2.destroyAllWindows()
        vis.destroy_window()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
