#!/usr/bin/env python3
"""订阅 camera_top 点云话题，持续运行 GraspNet 抓取检测并流式显示结果。

订阅 ``/camera/camera_top/depth/color/points`` (sensor_msgs/PointCloud2)，
回调只缓存最新一帧，后台线程运行 GraspNet 推理，主线程的 Open3D 窗口持续刷新点云与夹爪。
这样推理耗时不会卡住显示窗口，实现"流式"更新。

按键:
    Space - 暂停/继续推理
    Esc/Q - 退出

.. note::

    RealSense 发布的 PointCloud2 包含 xyz + rgb 字段，即为"标注后"的彩色点云。
    此脚本不依赖 cv_bridge 或 ros2_numpy，直接解析 PointCloud2 二进制数据。

Usage::

    ros2 run xtrainer_task graspnet_test --ros-args -p checkpoint_path:="./model/checkpoint.tar"
"""

import os
import sys
import threading
import time
import traceback

import numpy as np
import open3d as o3d
import rclpy
import torch
from ament_index_python.packages import get_package_share_directory
from graspnetAPI import GraspGroup
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

from graspnet.graspnet import GraspNet, pred_decode


class GraspNetTestNode(Node):
    """ROS2 节点：订阅点云话题 → 后台推理 → Open3D 流式显示。"""

    def __init__(self):
        super().__init__("graspnet_test")

        model_path = os.path.join(
            get_package_share_directory("graspnet"),
            "model",
            "checkpoint-rs.tar",
        )

        # ── 参数 ──────────────────────────────────────────────
        self.declare_parameter("pointcloud_topic", "/camera/camera_top/depth/color/points")
        self.declare_parameter("max_distance", 1.5)
        self.declare_parameter("checkpoint_path", model_path)
        self.declare_parameter("num_point", 20000)
        self.declare_parameter("num_view", 300)
        self.declare_parameter("collision_thresh", 0.01)
        self.declare_parameter("voxel_size", 0.01)
        self.declare_parameter("top_k_grasps", 5)
        self.declare_parameter("max_grasp_depth", 0.095)
        self.declare_parameter("min_infer_interval", 0.0)

        self._pointcloud_topic = self.get_parameter("pointcloud_topic").value
        self._max_distance = self.get_parameter("max_distance").value
        self._checkpoint_path = self.get_parameter("checkpoint_path").value
        self._num_point = self.get_parameter("num_point").value
        self._num_view = self.get_parameter("num_view").value
        self._collision_thresh = self.get_parameter("collision_thresh").value
        self._voxel_size = self.get_parameter("voxel_size").value
        self._top_k_grasps = self.get_parameter("top_k_grasps").value
        self._max_grasp_depth = self.get_parameter("max_grasp_depth").value
        self._min_infer_interval = self.get_parameter("min_infer_interval").value

        # ── 加载 GraspNet 模型 ──────────────────────────────────
        self._graspnet = None
        if self._checkpoint_path:
            self.get_logger().info(f"Loading GraspNet model from {self._checkpoint_path} ...")
            self._graspnet = self._load_graspnet()
            self.get_logger().info("GraspNet model loaded successfully.")
        else:
            self.get_logger().fatal("No checkpoint_path provided,")
            rclpy.shutdown()
            return

        # ── 流式状态 ──────────────────────────────────────────
        self._latest_msg = None
        self._msg_lock = threading.Lock()

        self._result_lock = threading.Lock()
        self._result_cloud = None
        self._result_geometries = None

        self._running = True
        self._paused = False

        # ── Open3D 几何体 (主线程维护) ────────────────────────
        self._vis = None
        self._cloud_geo = o3d.geometry.PointCloud()
        self._gripper_geos = []
        self._geo_added = False
        self._last_log = 0.0

        # ── 订阅点云话题 ───────────────────────────────────────
        self._sub = self.create_subscription(
            PointCloud2,
            self._pointcloud_topic,
            self._pointcloud_callback,
            10,
        )

        self.get_logger().info(f"Waiting for point cloud on '{self._pointcloud_topic}' ...")

        # ── 启动后台推理线程 ──────────────────────────────────
        self._worker = threading.Thread(target=self._inference_loop, daemon=True)
        self._worker.start()

    # ── 回调：只缓存最新帧 (非阻塞) ────────────────────────────
    def _pointcloud_callback(self, msg: PointCloud2):
        with self._msg_lock:
            self._latest_msg = msg

    # ── 后台推理线程 ──────────────────────────────────────────
    def _inference_loop(self):
        while self._running and rclpy.ok():
            if self._paused:
                time.sleep(0.05)
                continue

            with self._msg_lock:
                msg = self._latest_msg
                self._latest_msg = None

            if msg is None:
                time.sleep(0.02)
                continue

            t0 = time.time()
            try:
                cloud_o3d = self._ros_pointcloud_to_o3d(msg, self._max_distance)
                grippers = []
                gg = self._predict_grasps(cloud_o3d)
                if gg is not None and len(gg) > 0:
                    grippers = self._filter_and_vis_grasps(gg)

                # 把结果交给主线程显示
                with self._result_lock:
                    self._result_cloud = cloud_o3d
                    self._result_geometries = grippers

                now = time.time()
                if now - self._last_log > 1.0:
                    self._last_log = now
                    self.get_logger().info(
                        f"frame: {len(cloud_o3d.points)} pts, "
                        f"{len(grippers)} grasps, {now - t0:.2f}s"
                    )
            except Exception:
                self.get_logger().error(f"Frame processing failed:\n{traceback.format_exc()}")

            # 可选：限制推理频率
            dt = time.time() - t0
            if self._min_infer_interval > dt:
                time.sleep(self._min_infer_interval - dt)

    # ── 主线程：刷新一帧显示 ──────────────────────────────────
    def attach_visualizer(self, vis):
        self._vis = vis

    def update_frame(self) -> bool:
        """消费最新推理结果并刷新窗口。返回 False 表示窗口已关闭。"""
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

            # 移除旧夹爪
            for g in self._gripper_geos:
                self._vis.remove_geometry(g, reset_bounding_box=False)
            self._gripper_geos = list(grippers)
            for g in self._gripper_geos:
                self._vis.add_geometry(g, reset_bounding_box=False)

        ok = self._vis.poll_events()
        self._vis.update_renderer()
        return ok

    @staticmethod
    def _ros_pointcloud_to_o3d(msg: PointCloud2, max_distance: float = 1.5) -> o3d.geometry.PointCloud:
        """将 sensor_msgs/PointCloud2 转换为 Open3D PointCloud。

        直接解析 PointCloud2 中的 raw data buffer，避免结构化 numpy 数组类型问题。

        Parameters
        ----------
        msg : PointCloud2
            ROS 点云消息
        max_distance : float
            点云距离上限 (米), 默认 1.5m, 过滤超过此距离的点

        Returns
        -------
        o3d.geometry.PointCloud
            Open3D 点云对象
        """
        # ── 解析字段布局 ──────────────────────────────────────
        field_names = [f.name for f in msg.fields]
        if "x" not in field_names or "y" not in field_names or "z" not in field_names:
            raise ValueError(f"PointCloud2 missing x/y/z fields, got {field_names}")

        # 每个点的字节数
        point_step = msg.point_step

        # 获取各字段的 offset
        offsets = {}
        dtype_map = {
            1: np.int8,
            2: np.int16,
            4: np.int32,
            5: np.uint16,
            6: np.uint32,
            7: np.float32,
            8: np.float64,
        }
        for f in msg.fields:
            np_dtype = dtype_map.get(f.datatype, np.float32)
            offsets[f.name] = (f.offset, np_dtype)

        # ── 读取全部点云原始字节 ──────────────────────────────
        raw = np.frombuffer(msg.data, dtype=np.uint8)
        num_points = len(raw) // point_step
        raw = raw[: num_points * point_step].reshape(num_points, point_step)

        # ── 提取 X, Y, Z ──────────────────────────────────────
        def get_field(arr, name):
            off, dt = offsets[name]
            return np.frombuffer(arr[:, off: off + np.dtype(dt).itemsize].tobytes(), dtype=dt).astype(np.float32)

        x = get_field(raw, "x")
        y = get_field(raw, "y")
        z = get_field(raw, "z")

        # 过滤 NaN 与距离
        valid = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
        if max_distance > 0:
            dist = np.sqrt(x**2 + y**2 + z**2)
            valid &= (dist <= max_distance)
        x, y, z = x[valid], y[valid], z[valid]

        xyz = np.column_stack([x, y, z])

        if len(xyz) == 0:
            raise ValueError("No valid points in point cloud")

        cloud = o3d.geometry.PointCloud()
        cloud.points = o3d.utility.Vector3dVector(xyz)

        # ── 提取 RGB (如果存在) ─────────────────────────────────
        has_rgb = "rgb" in field_names or "rgba" in field_names
        if has_rgb:
            rgb_field = "rgb" if "rgb" in field_names else "rgba"
            off, dt = offsets[rgb_field]
            rgb_raw = raw[:, off: off + np.dtype(dt).itemsize].tobytes()
            # RGB 是 packed float32 (IEEE 754 bits → uint8 R,G,B)
            if dt == np.float32:
                packed = np.frombuffer(rgb_raw, dtype=np.float32)[valid]
                rgb_bytes = packed.view(np.uint32)
            else:
                packed = np.frombuffer(rgb_raw, dtype=dt)[valid].astype(np.uint32)

            r = ((rgb_bytes >> 16) & 0xFF).astype(np.float32) / 255.0
            g = ((rgb_bytes >> 8) & 0xFF).astype(np.float32) / 255.0
            b = (rgb_bytes & 0xFF).astype(np.float32) / 255.0

            colors_rgb = np.column_stack([r, g, b])
            cloud.colors = o3d.utility.Vector3dVector(colors_rgb)

        return cloud

    def _load_graspnet(self) -> GraspNet:
        """加载 GraspNet 模型。"""
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
        """从 Open3D 点云预测抓取。

        Parameters
        ----------
        cloud_o3d : o3d.geometry.PointCloud
            输入点云

        Returns
        -------
        GraspGroup or None
            预测的抓取集合
        """
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
        color_sampled = colors[idxs]

        # ── 前向推理 ─────────────────────────────────────────
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
        gg = GraspGroup(gg_array)
        return gg

    def _filter_and_vis_grasps(self, gg):
        """过滤抓取并生成可视化几何体。

        Parameters
        ----------
        gg : GraspGroup
            原始抓取集合

        Returns
        -------
        list
            Open3D 三角网格列表
        """
        # 过滤夹爪长度(depth) > max_grasp_depth 的抓取
        if self._max_grasp_depth > 0 and len(gg) > 0:
            keep = np.where(gg.depths <= self._max_grasp_depth)[0]
            gg = gg[keep]
        gg.nms()
        gg.sort_by_score()
        gg = gg[: self._top_k_grasps]
        return gg.to_open3d_geometry_list()


def main():
    rclpy.init(args=sys.argv)

    node = GraspNetTestNode()
    if not node._graspnet:
        return

    # ── 创建持久可视化窗口 ──────────────────────────────────
    vis = o3d.visualization.VisualizerWithKeyCallback()
    vis.create_window(
        window_name="GraspNet Streaming - Camera Top Point Cloud",
        width=1280,
        height=720,
    )
    node.attach_visualizer(vis)

    # ── 按键回调 ────────────────────────────────────────────
    def _on_quit(_vis):
        node.get_logger().info("Quit requested.")
        node._running = False
        return False

    def _on_pause(_vis):
        node._paused = not node._paused
        node.get_logger().info("Paused" if node._paused else "Resumed")
        return True

    vis.register_key_callback(27, _on_quit)          # Esc
    vis.register_key_callback(ord("Q"), _on_quit)    # Q
    vis.register_key_callback(32, _on_pause)         # Space

    # ── 主循环：spin + 刷新显示 ────────────────────────────
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)

    try:
        while rclpy.ok() and node._running:
            executor.spin_once(timeout_sec=0.01)
            if not node.update_frame():
                node.get_logger().info("Visualization window closed, shutting down ...")
                break
    except KeyboardInterrupt:
        pass
    finally:
        node._running = False
        if node._worker.is_alive():
            node._worker.join(timeout=2.0)
        vis.destroy_window()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
