#!/usr/bin/env python3
"""Grounding DINO + SAM2 白色桌面检测 → ROI → SAM2 自动掩码 → 物体点云 + GraspNet。

流程:
    1. 订阅 ``/camera/camera_top/color/image_raw`` (彩色图) 与
       ``/camera/camera_top/depth/color/points`` (有序彩色点云)。
    2. Grounding DINO 识别 ``prompt`` (默认 "white square.")。
    3. SAM2 对每个 detection box 切分, 得到白色桌面的 2D mask。
    4. 由 mask 计算桌面 ROI (外接矩形), 可用 ``roi_shrink_px`` 参数内缩。
    5. 在 ROI 裁剪区域内运行 ``SAM2AutomaticMaskGenerator``, 生成桌面上的
       物体掩码 (按面积降序, #0 通常是桌子表面本身)。
    6. 点云切分: 把 **除桌子 mask (面积最大) 以外** 的物体 mask 并集应用到
       点云, 得到"只有物体"的子点云, 单独显示在 Open3D 窗口。
    7. 在物体子点云上运行 GraspNet 生成抓取姿态, 在 Open3D 中显示夹爪。

显示:
    - OpenCV 窗口 "Desk Object Detect":
        - 绿色半透明: 白色桌面 SAM2 mask
        - 黄色矩形: ROI (已按 ``roi_shrink_px`` 内缩)
        - 蓝色: 物体掩码轮廓 + bbox
        - 青色半透明: 实际送入 GraspNet 的物体区域 (排除桌子 mask)
    - Open3D 窗口: 物体子点云 + GraspNet 夹爪 + 每抓取坐标系 (流式刷新)

按键:
    OpenCV 窗口按 ``q`` 或 Open3D 窗口按 ``Esc``/``Q`` 退出。

Boot::

    export PYTHONPATH=/opt/Project/Grounded-SAM-2/grounding_dino:/opt/Project/Grounded-SAM-2
    ros2 run xtrainer_task detect_desk_object --ros-args \
        -p prompt:="white square." \
        -p roi_shrink_px:=0 \
        -p drop_largest_mask:=true \
        -p pointcloud_topic:=/camera/camera_top/depth/color/points

.. note::

    不依赖 ``cv_bridge``, 直接解析 ROS 原始消息 (同 ``graspnet_sam_region_test.py``)。
    推断在后台线程执行, Open3D 窗口在主线程刷新, 相互不阻塞。
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
from sam2.automatic_mask_generator import SAM2AutomaticMaskGenerator
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


class DetectDeskObjectNode(Node):
    """ROS2 节点: 白色桌面检测 → ROI → 物体点云分割 → GraspNet 抓取检测。"""

    def __init__(self):
        super().__init__("detect_desk_object")

        model_path = os.path.join(
            get_package_share_directory("graspnet"),
            "model",
            "checkpoint-rs.tar",
        )

        # ── 参数 ──────────────────────────────────────────────
        self.declare_parameter("image_topic", "/camera/camera_top/color/image_raw")
        self.declare_parameter("pointcloud_topic", "/camera/camera_top/depth/color/points")
        self.declare_parameter("prompt", "white square.")
        self.declare_parameter("box_threshold", 0.35)
        self.declare_parameter("text_threshold", 0.25)
        self.declare_parameter("skip_frames", 0)  # 每隔 N 帧检测一次, 0=每帧
        # SAM2 模型选择 (6GB 显存下用 base_plus 比 large 省 ~0.5GB, 留内存给 GraspNet)
        self.declare_parameter(
            "sam2_checkpoint",
            "/opt/Project/Grounded-SAM-2/checkpoints/sam2.1_hiera_base_plus.pt",
        )
        self.declare_parameter("sam2_config", "configs/sam2.1/sam2.1_hiera_b+.yaml")
        # ROI 整体向内缩 x 像素 (正值缩小, 负值扩大)
        self.declare_parameter("roi_shrink_px", 0)
        # 是否在 ROI 内运行 SAM2 自动掩码生成 (False 只显示桌面 mask + ROI)
        self.declare_parameter("run_auto_masks", True)
        # 是否排除面积最大的 mask (桌子表面本身), 只保留桌上的物体
        self.declare_parameter("drop_largest_mask", True)
        # SAM2AutomaticMaskGenerator 参数
        self.declare_parameter("amg_points_per_side", 16)
        self.declare_parameter("amg_pred_iou_thresh", 0.8)
        self.declare_parameter("amg_stability_score_thresh", 0.95)
        self.declare_parameter("amg_min_mask_region_area", 0)
        self.declare_parameter("amg_box_nms_thresh", 0.7)
        # 自动掩码面积下限 (相对 ROI 面积比例 0~1), 过滤噪点
        self.declare_parameter("min_area_ratio", 0.002)
        # 点云距离上限 (m), 过滤远处噪声
        self.declare_parameter("max_distance", 1.5)
        # GraspNet 参数
        self.declare_parameter("checkpoint_path", model_path)
        self.declare_parameter("num_point", 20000)
        self.declare_parameter("num_view", 300)
        self.declare_parameter("collision_thresh", 0.01)
        self.declare_parameter("voxel_size", 0.01)
        self.declare_parameter("top_k_grasps", 5)
        # 夹爪长度(depth)上限, 单位 m, 过滤 depth>此值的抓取。9.5cm=0.095
        self.declare_parameter("max_grasp_depth", 0.095)
        self.declare_parameter("min_infer_interval", 0.0)

        self._image_topic = self.get_parameter("image_topic").value
        self._pointcloud_topic = self.get_parameter("pointcloud_topic").value
        self._prompt = self.get_parameter("prompt").value
        self._skip_frames = self.get_parameter("skip_frames").value
        self._roi_shrink = self.get_parameter("roi_shrink_px").value
        self._use_auto_masks = self.get_parameter("run_auto_masks").value
        self._drop_largest = self.get_parameter("drop_largest_mask").value
        self._amg_points_per_side = self.get_parameter("amg_points_per_side").value
        self._amg_pred_iou_thresh = self.get_parameter("amg_pred_iou_thresh").value
        self._amg_stability = self.get_parameter("amg_stability_score_thresh").value
        self._amg_min_area = self.get_parameter("amg_min_mask_region_area").value
        self._amg_nms = self.get_parameter("amg_box_nms_thresh").value
        self._min_area_ratio = self.get_parameter("min_area_ratio").value
        self._max_distance = self.get_parameter("max_distance").value
        self._checkpoint_path = self.get_parameter("checkpoint_path").value
        self._num_point = self.get_parameter("num_point").value
        self._num_view = self.get_parameter("num_view").value
        self._collision_thresh = self.get_parameter("collision_thresh").value
        self._voxel_size = self.get_parameter("voxel_size").value
        self._top_k_grasps = self.get_parameter("top_k_grasps").value
        self._max_grasp_depth = self.get_parameter("max_grasp_depth").value
        self._min_infer_interval = self.get_parameter("min_infer_interval").value

        # ── 加载 Grounding DINO + SAM2 ────────────────────────
        self.get_logger().info("Loading Grounding DINO + SAM2 models …")
        self._wrapper = DinoWrapper(
            box_threshold=self.get_parameter("box_threshold").value,
            text_threshold=self.get_parameter("text_threshold").value,
            sam2_checkpoint=self.get_parameter("sam2_checkpoint").value,
            sam2_config=self.get_parameter("sam2_config").value,
        )

        # ── 加载 SAM2 自动掩码生成器 (复用同一 SAM2 权重) ──────
        self._amg = None
        if self._use_auto_masks:
            self._amg = SAM2AutomaticMaskGenerator(
                model=self._wrapper.sam2_model,
                points_per_side=self._amg_points_per_side,
                pred_iou_thresh=self._amg_pred_iou_thresh,
                stability_score_thresh=self._amg_stability,
                min_mask_region_area=self._amg_min_area,
                box_nms_thresh=self._amg_nms,
                output_mode="binary_mask",
            )
        self.get_logger().info("Detection models loaded.")

        # ── 加载 GraspNet ──────────────────────────────────────
        if self._checkpoint_path:
            self.get_logger().info(f"Loading GraspNet model from {self._checkpoint_path} …")
            self._graspnet = self._load_graspnet()
            self.get_logger().info("GraspNet model loaded successfully.")
        else:
            self.get_logger().fatal("No checkpoint_path provided.")
            self._graspnet = None

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

        # ── 帧计数 (skip_frames 用) ───────────────────────────
        self._frame_count = 0

        # ── 订阅 ───────────────────────────────────────────────
        self.create_subscription(Image, self._image_topic, self._image_callback, 10)
        self.create_subscription(
            PointCloud2, self._pointcloud_topic, self._cloud_callback, 10
        )

        self.get_logger().info(
            f'Subscribed image="{self._image_topic}", cloud="{self._pointcloud_topic}", '
            f'prompt="{self._prompt}", roi_shrink_px={self._roi_shrink}, '
            f"drop_largest_mask={self._drop_largest}. "
            "Press 'q' in the OpenCV window or Esc/Q in the Open3D window to exit."
        )

        # ── 启动推理线程 ──────────────────────────────────────
        self._worker = threading.Thread(target=self._inference_loop, daemon=True)
        self._worker.start()

    # ── 回调: 只缓存最新帧 ─────────────────────────────────────
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
                if image_msg is not None and cloud_msg is not None:
                    self._latest_cloud = None

            if image_msg is None or cloud_msg is None:
                time.sleep(0.02)
                continue

            self._frame_count += 1
            # 跳帧: 每隔 skip_frames+1 帧才处理一次
            if self._skip_frames > 0 and self._frame_count % (self._skip_frames + 1) != 1:
                time.sleep(0.02)
                continue

            t0 = time.time()
            try:
                self._process_frame(image_msg, cloud_msg)
            except Exception:
                self.get_logger().error(
                    f"Frame processing failed:\n{traceback.format_exc()}"
                )

            dt = time.time() - t0
            if self._min_infer_interval > dt:
                time.sleep(self._min_infer_interval - dt)

    # ── 单帧处理 ──────────────────────────────────────────────
    def _process_frame(self, image_msg: Image, cloud_msg: PointCloud2):
        t0 = time.time()
        prompt = self.get_parameter("prompt").value
        self._prompt = prompt  # 热更新 prompt

        # ── 1. 彩色图 → BGR ──────────────────────────────────
        frame = ros_image_to_cv2(image_msg)

        # ── 2. GroundingDINO + SAM2 检测桌面 ─────────────────
        det = self._wrapper.detect(frame, prompt)
        H, W = frame.shape[:2]
        table_mask, roi = self._compute_table_roi(det, H, W)

        # ── 3. ROI 内 SAM2 自动掩码生成 (桌子上的物体) ────────
        anns = []
        if self._use_auto_masks and self._amg is not None and roi is not None:
            anns = self._run_auto_masks(frame, roi)

        # ── 4. 点云切分: 除桌子 mask 以外的物体点云 ───────────
        cloud_o3d = None
        object_mask = None
        n_pts = 0
        grippers, axes = [], []
        try:
            xyz, rgb, H_pc, W_pc = self._ros_pointcloud_to_organized(cloud_msg)
            object_mask = self._build_object_mask(anns, H_pc, W_pc)
            if object_mask is not None:
                valid = np.isfinite(xyz).all(axis=2)
                if self._max_distance > 0:
                    valid &= (np.linalg.norm(xyz, axis=2) <= self._max_distance)
                sel = object_mask & valid
                pts = xyz[sel]
                cols = rgb[sel]
                n_pts = len(pts)
                if n_pts > 0:
                    cloud_o3d = o3d.geometry.PointCloud()
                    cloud_o3d.points = o3d.utility.Vector3dVector(pts)
                    cloud_o3d.colors = o3d.utility.Vector3dVector(cols)

                    # ── 5. GraspNet 抓取检测 ──────────────────
                    gg = self._predict_grasps(cloud_o3d)
                    if gg is not None and len(gg) > 0:
                        grippers, axes = self._filter_and_vis_grasps(gg)
                        self._log_grasps(gg)
        except Exception:
            self.get_logger().error(
                f"[3D pointcloud/GraspNet] failed:\n{traceback.format_exc()}"
            )

        # ── 6. 结果交给主线程刷新 Open3D ─────────────────────
        with self._result_lock:
            self._result_cloud = cloud_o3d
            self._result_geometries = grippers + axes

        # ── 7. 2D 结果 OpenCV 显示 ───────────────────────────
        display = self._draw(frame, table_mask, roi, anns, object_mask)
        cv2.putText(
            display,
            f"dets={len(det.boxes)} roi={'yes' if roi is not None else 'no'} "
            f"objects={len(anns)} pts={n_pts} grasps={len(grippers)}",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2,
        )
        cv2.imshow("Desk Object Detect", display)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            self.get_logger().info("Pressed 'q' in OpenCV window, exiting …")
            self._running = False

        if anns:
            self._log_auto_masks(anns)

        now = time.time()
        if now - self._last_log > 1.0:
            self._last_log = now
            self.get_logger().info(
                f"dets={len(det.boxes)} roi={'yes' if roi is not None else 'no'} "
                f"objects={len(anns)} grasps={len(grippers)} ({now - t0:.2f}s)"
            )

    # ── 由白色桌面 mask 计算 ROI ─────────────────────────────
    def _compute_table_roi(self, det, H, W):
        """由检测结果的 mask 计算白色桌面 ROI。

        Returns
        -------
        tuple[np.ndarray, tuple] or tuple[None, None]
            (mask2d, (x0, y0, x1, y1)); mask2d 为 (H, W) bool 的桌面掩码,
            roi 为按 ``roi_shrink_px`` 内缩后的外接矩形 (全图坐标, 已 clamp)。
        """
        if len(det.boxes) == 0 or len(det.masks) == 0:
            return None, None

        mask2d = np.any(det.masks, axis=0)
        if not np.any(mask2d):
            return None, None

        ys, xs = np.where(mask2d)
        x0, y0, x1, y1 = xs.min(), ys.min(), xs.max(), ys.max()

        s = int(self._roi_shrink)
        x0, y0 = x0 + s, y0 + s
        x1, y1 = x1 - s, y1 - s

        x0 = max(0, min(W - 1, x0))
        y0 = max(0, min(H - 1, y0))
        x1 = max(x0 + 1, min(W - 1, x1))
        y1 = max(y0 + 1, min(H - 1, y1))
        return mask2d, (x0, y0, x1, y1)

    # ── 在 ROI 内运行 SAM2 自动掩码生成 ──────────────────────
    def _run_auto_masks(self, frame, roi):
        """在 ROI 裁剪区域内运行 SAM2AutomaticMaskGenerator。

        Parameters
        ----------
        frame : np.ndarray
            BGR 全图
        roi : tuple
            (x0, y0, x1, y1) 全图坐标

        Returns
        -------
        list[dict]
            每个元素含:
                - segmentation_full: (H, W) bool 全图坐标掩码
                - bbox_xyxy_full: (x1, y1, x2, y2) 全图坐标 bbox
                - bbox (ROI 局部 XYWH), area, predicted_iou, stability_score
            按面积降序排列 (#0 通常是桌子表面本身)。
        """
        x0, y0, x1, y1 = roi
        crop = frame[y0:y1 + 1, x0:x1 + 1]
        roi_area = crop.shape[0] * crop.shape[1]
        min_area = int(self._min_area_ratio * roi_area)

        anns = self._amg.generate(crop)

        out = []
        for a in anns:
            if a["area"] < min_area:
                continue
            bx, by, bw, bh = a["bbox"]
            a["bbox_xyxy_full"] = [x0 + bx, y0 + by, x0 + bx + bw, y0 + by + bh]
            full = np.zeros((frame.shape[0], frame.shape[1]), dtype=bool)
            full[y0:y1 + 1, x0:x1 + 1] = a["segmentation"]
            a["segmentation_full"] = full
            out.append(a)

        # 按面积降序, 大物体在前
        out.sort(key=lambda a: a["area"], reverse=True)
        return out

    # ── 由物体 mask 构建点云选择掩码 (排除桌子) ──────────────
    def _build_object_mask(self, anns, H_pc, W_pc):
        """把除了桌子 (面积最大) 以外的物体 mask 并集 → 点云尺寸的选择掩码。

        Returns
        -------
        np.ndarray or None
            (H_pc, W_pc) bool, True=保留对应点云 (桌面上的物体); 无物体时返回 None。
        """
        if not anns:
            return None

        start = 1 if self._drop_largest else 0
        if start >= len(anns):
            return None

        combined = np.any(
            [a["segmentation_full"] for a in anns[start:]], axis=0
        )
        if not np.any(combined):
            return None

        if combined.shape != (H_pc, W_pc):
            combined = cv2.resize(
                combined.astype(np.uint8),
                (W_pc, H_pc),
                interpolation=cv2.INTER_NEAREST,
            ).astype(bool)
        return combined

    # ── 日志输出自动掩码结果 ─────────────────────────────────
    def _log_auto_masks(self, anns):
        self.get_logger().info(f"—— SAM automatic masks in ROI: {len(anns)} ——")
        for i, a in enumerate(anns):
            bx1, by1, bx2, by2 = a["bbox_xyxy_full"]
            tag = " (table)" if (i == 0 and self._drop_largest) else ""
            self.get_logger().info(
                f"#{i}: bbox=({bx1:.0f},{by1:.0f})-({bx2:.0f},{by2:.0f}) "
                f"area={a['area']} iou={a['predicted_iou']:.3f} "
                f"stab={a['stability_score']:.3f}{tag}"
            )

    # ── OpenCV 可视化 ────────────────────────────────────────
    def _draw(self, display, table_mask, roi, anns, object_mask=None):
        if table_mask is not None:
            overlay = display.copy()
            overlay[table_mask] = (0, 255, 0)
            display = cv2.addWeighted(display, 0.7, overlay, 0.3, 0)

        if roi is not None:
            x0, y0, x1, y1 = roi
            cv2.rectangle(display, (x0, y0), (x1, y1), (0, 255, 255), 2)

        for i, a in enumerate(anns):
            mask = a["segmentation_full"]
            mask_u8 = mask.astype(np.uint8)
            cnts, _ = cv2.findContours(
                mask_u8, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )
            cv2.drawContours(display, cnts, -1, (255, 0, 0), 2)

            bx1, by1, bx2, by2 = [int(v) for v in a["bbox_xyxy_full"]]
            cv2.rectangle(display, (bx1, by1), (bx2, by2), (255, 128, 0), 1)
            cx = int((bx1 + bx2) / 2)
            cy = int((by1 + by2) / 2)
            cv2.putText(
                display, f"{i}", (cx, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                (255, 255, 0), 2,
            )

        # 青色半透明: 实际送入 GraspNet 的物体区域 (排除桌子)
        if object_mask is not None:
            overlay = display.copy()
            overlay[object_mask] = (255, 128, 0)
            display = cv2.addWeighted(display, 0.7, overlay, 0.3, 0)

        return display

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
            (夹爪三角网格列表, 每抓取坐标系列表)
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

    def _log_grasps(self, gg):
        """将 GraspNet 输出的抓取姿态打印到终端。"""
        n = min(len(gg), self._top_k_grasps)
        self.get_logger().info(f"—— GraspNet top-{n} grasps ——")
        for i in range(n):
            g = gg[i]
            t = g.translation
            rot = g.rotation_matrix
            self.get_logger().info(
                f"#{i}: score={g.score:.3f} width={g.width * 100:.1f}cm "
                f"depth={g.depth * 100:.1f}cm\n"
                f"   t=({t[0]:.4f}, {t[1]:.4f}, {t[2]:.4f})\n"
                f"   R=[{rot[0,0]:.3f} {rot[0,1]:.3f} {rot[0,2]:.3f}; "
                f"{rot[1,0]:.3f} {rot[1,1]:.3f} {rot[1,2]:.3f}; "
                f"{rot[2,0]:.3f} {rot[2,1]:.3f} {rot[2,2]:.3f}]"
            )

    # ── 主线程: 刷新 Open3D 显示 ──────────────────────────────
    def attach_visualizer(self, vis):
        self._vis = vis

    def update_frame(self) -> bool:
        """消费最新推理结果并刷新 Open3D 窗口。返回 False 表示窗口已关闭。"""
        if self._vis is None:
            return True

        have_update = False
        cloud = None
        geometries = None
        with self._result_lock:
            if self._result_cloud is not None or self._result_geometries is not None:
                cloud = self._result_cloud
                geometries = self._result_geometries or []
                self._result_cloud = None
                self._result_geometries = None
                have_update = True

        if have_update:
            # 清空旧的抓取几何体
            for g in self._gripper_geos:
                self._vis.remove_geometry(g, reset_bounding_box=False)
            self._gripper_geos = list(geometries)

            if cloud is not None:
                self._cloud_geo.points = cloud.points
                self._cloud_geo.colors = cloud.colors
                if not self._geo_added:
                    self._vis.add_geometry(self._cloud_geo, reset_bounding_box=True)
                    self._geo_added = True
                else:
                    self._vis.update_geometry(self._cloud_geo)
                for g in self._gripper_geos:
                    self._vis.add_geometry(g, reset_bounding_box=False)
            else:
                # 无物体点云时清空显示
                if self._geo_added:
                    self._vis.remove_geometry(self._cloud_geo, reset_bounding_box=False)
                    self._geo_added = False

        ok = self._vis.poll_events()
        self._vis.update_renderer()
        return ok


def main():
    rclpy.init(args=sys.argv)

    node = DetectDeskObjectNode()
    if node._graspnet is None:
        node.destroy_node()
        rclpy.shutdown()
        return

    # ── Open3D 持久窗口 ────────────────────────────────────
    vis = o3d.visualization.VisualizerWithKeyCallback()
    vis.create_window(
        window_name="Desk Objects + GraspNet - Segmented Point Cloud",
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
