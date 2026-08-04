"""Entry point for xtrainer_task — robot movement using MoveIt.

Configs (URDF, SRDF, ompl_planning, etc.) are expected to be loaded
from ROS 2 parameters, which are set by the launch file:
    ros2 launch xtrainer_task start.launch.py

Usage
-----
    ros2 launch xtrainer_task start.launch.py
"""


import math
import os
import sys
import threading
import time
from collections import deque
from typing import Dict, Optional, Tuple

import cv2
import numpy as np
import open3d as o3d
import rclpy
import tf2_geometry_msgs
import tf2_ros
import torch
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PointStamped, Pose, PoseStamped
from graspnetAPI import GraspGroup
from moveit.planning import MoveItPy
from rclpy.callback_groups import (MutuallyExclusiveCallbackGroup,
                                   ReentrantCallbackGroup)
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from visualization_msgs.msg import Marker

from graspnet.graspnet import GraspNet, pred_decode
from xtrainer_gripper.gripper_control import GripperController
from xtrainer_task.dino_wrapper import DinoWrapper
from xtrainer_task.robot_move import Planner, RobotMover

_COLOR_ENCODINGS = {"bgr8", "rgb8"}
_DEPTH_ENCODINGS = {"16UC1", "mono16"}

# 彩色 / 对齐深度话题模板
_COLOR_TOPIC_TEMPLATE = "/camera/{name}/color/image_raw"
_DEPTH_TOPIC_TEMPLATE = "/camera/{name}/aligned_depth_to_color/image_raw"

_CAMERA_NAMES = ("camera_top", "camera_left", "camera_right")

_CAMERA_OPTICAL_FRAMES: Dict[str, str] = {
    "camera_top": "camera_top_color_optical_frame",
    "camera_left": "camera_left_color_optical_frame",
    "camera_right": "camera_right_color_optical_frame",
}
_ARM_TIP = {
    "Arm1": "L1_gripper_tip",
    "Arm2": "L2_gripper_tip",
}
_ARM_CAM = {
    "Arm1": "camera_left",
    "Arm2": "camera_right",
}
_GRIPPER = {
    "Arm1": "left",
    "Arm2": "right",
}

# ── Step 2 Any Grasp: GroundedSAM2 + GraspNet ──────────────────────────
_GRASP_PC_TOPIC_TEMPLATE = "/camera/{name}/depth/color/points"
_GRASP_PROMPT = "bottle."
_GRASP_NUM_POINT = 20000
_GRASP_NUM_VIEW = 300
# 夹爪长度(depth)上限, 单位 m, 过滤 depth>此值的抓取。9.5cm=0.095
_GRASP_MAX_DEPTH = 0.095
_GRASP_TOP_K = 10
_GRASP_MIN_POINTS = 50

# GraspNet 夹爪坐标系 → 机械臂 tip frame 的轴重映射 (列置换)。
#   graspnet: +X=接近方向, +Y=手指开合, +Z=掌心法向
#   robot:    +Z=接近方向, +X=手指开合, +Y=其余
# R_tool = R_grasp @ _GRASP_TO_TOOL, 使 robot +Z←grasp +X, robot +X←grasp +Y。
_GRASP_TO_TOOL = np.array(
    [[0.0, 0.0, 1.0],
     [1.0, 0.0, 0.0],
     [0.0, 1.0, 0.0]],
    dtype=np.float64,
)

# PointCloud2 datatype → numpy
_DTYPE_MAP = {
    1: np.int8,
    2: np.int16,
    4: np.int32,
    5: np.uint16,
    6: np.uint32,
    7: np.float32,
    8: np.float64,
}

class XTrainerTask(Node):
    def __init__(self):
        super().__init__("xtrainer_task")

        # SAM 边缘膨胀: 在 3D 空间把分割点云往外多保留 radius_m 内的点
        self.declare_parameter("mask_dilation_m", 0.04)
        self._mask_dilation_m = float(
            self.get_parameter("mask_dilation_m").value
        )

        # MoveItPy reads configs from the parameter server (set by launch file)
        self._moveit = MoveItPy(node_name='xtrainer_task_moveit')
        self.mover = RobotMover(self._moveit, self)

        # Give MoveIt some time to receive latest /joint_states and TF
        self.get_logger().info('Waiting for MoveIt state to populate …')
        time.sleep(2.0)

        # DINO 检测器
        self.dino = DinoWrapper(
            device='cuda', box_threshold=0.35, text_threshold=0.25,
        )

        # 夹爪
        self.gripper = GripperController(self)

        # ── 加载 GraspNet (抓取检测, 阻塞, 只执行一次) ──
        model_path = os.path.join(
            get_package_share_directory("graspnet"),
            "model",
            "checkpoint-rs.tar",
        )
        self.get_logger().info(f"Loading GraspNet model from {model_path} …")
        self._graspnet = self._load_graspnet(model_path)
        self.get_logger().info("GraspNet model loaded successfully.")

        # ── 各相机最新帧快照 (单锁, 每帧覆盖) ──
        self._snapshots: Dict[str, dict] = {}
        self._snapshots_lock = threading.Lock()

        # ── 相机内参缓存 (camera_name → CameraInfo) ──
        self._camera_infos: Dict[str, CameraInfo] = {}
        self._camera_info_lock = threading.Lock()

        # ── 持久 Open3D 可视化 (单窗口复用) ──
        self._vis_lock = threading.Lock()
        self._vis_pending = None
        self._vis_thread = None

        # ── TF2 缓冲与监听器 ──
        # cache_time 调大到 120s: Step2 的 SAM2+GraspNet 单轮推理 ~15s,
        # 若 TF 失败还会进入 while 循环重试, 累积可达数十秒. 默认 10s cache
        # 会在 SAM/GraspNet 完成前就把点云采集时刻的 TF 丢弃, 导致
        # "Lookup would require extrapolation into the past".
        self._tf_buffer = tf2_ros.Buffer(
            cache_time=rclpy.duration.Duration(seconds=120)
        )
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._camera_cb_group = ReentrantCallbackGroup()   # 相机回调组，允许并发/独立于 launch
        self._launch_cb_group = MutuallyExclusiveCallbackGroup()  # launch 定时器单独一组


        # 为三个相机分别订阅彩色、对齐深度 & 内参话题
        for name in _CAMERA_NAMES:
            color_topic = _COLOR_TOPIC_TEMPLATE.format(name=name)
            self.create_subscription(
                Image, color_topic,
                lambda msg, cam=name: self._color_callback(cam, msg),
                10,
                callback_group=self._camera_cb_group,
            )
            self.get_logger().info(f"Subscribed color: {color_topic}")

            depth_topic = _DEPTH_TOPIC_TEMPLATE.format(name=name)
            self.create_subscription(
                Image, depth_topic,
                lambda msg, cam=name: self._depth_callback(cam, msg),
                10,
                callback_group=self._camera_cb_group,
            )
            self.get_logger().info(f"Subscribed depth: {depth_topic}")

            info_topic = f"/camera/{name}/color/camera_info"
            self.create_subscription(
                CameraInfo, info_topic,
                lambda msg, cam=name: self._camera_info_callback(cam, msg),
                10,
                callback_group=self._camera_cb_group,
            )
            self.get_logger().info(f"Subscribed camera_info: {info_topic}")

            cloud_topic = _GRASP_PC_TOPIC_TEMPLATE.format(name=name)
            self.create_subscription(
                PointCloud2, cloud_topic,
                lambda msg, cam=name: self._cloud_callback(cam, msg),
                10,
                callback_group=self._camera_cb_group,
            )
            self.get_logger().info(f"Subscribed pointcloud: {cloud_topic}")

        # ── 检测点 Marker 发布器 (RViz 可视化) ──
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._marker_pub = self.create_publisher(Marker, '/detection_marker', qos)

        self.get_logger().info('#################### All module initialized ######################')

        self._launch_timer = self.create_timer(
            1.0,
            self._on_launch_timer,
            callback_group=self._launch_cb_group
        )

    # ------------------------------------------------------------------
    # Controll Task ####################################################
    # ------------------------------------------------------------------

    def _on_launch_timer(self):
        """One-shot timer callback — cancels timer after first invocation."""
        self._launch_timer.cancel()
        self.launch()

    def launch(self):

        self.gripper.open_both()

        pick_arm = "Arm1"

        """
        Step 1: Find Object and Approach
        """

        # Panning Loop
        while rclpy.ok():
            # Capture Loop
            while rclpy.ok():
                image_top = self.get_latest_color("camera_top")
                
                if image_top is None:
                    self.get_logger().error("No top camera image available.")
                    time.sleep(0.05)
                    continue

                result = self.dino.detect(image_top, "bottle")

                if len(result.boxes) == 0:
                    self.get_logger().warn("No bottle detected in top camera.")
                    cv2.imshow("Detection Result", image_top)
                    cv2.waitKey(1)
                    time.sleep(0.05)
                    continue

                try:
                    annotated_image = self.dino.annotate(image_top, result, draw_mask=True)
                except Exception as ex:
                    self.get_logger().warn("Bottle annotate failed, showing original image")
                    self.get_logger().log(ex)
                    annotated_image = image_top

                cv2.imshow("Detection Result", annotated_image)
                cv2.waitKey(10)

                break

            mid_x = (result.boxes[0, 0] + result.boxes[0, 2]) / 2
            mid_y = (result.boxes[0, 1] + result.boxes[0, 3]) / 2

            self.get_logger().info(f"Mid pixel: ({mid_x}, {mid_y})")
            
            mid_coordinate = self.pixel_to_base_link("camera_top", mid_x, mid_y)
            
            if mid_coordinate is None:
                self.get_logger().error("Failed to compute 3D coordinate of bottle center.")
                continue

            self._publish_detection_marker(mid_coordinate)

            pose_prepare = Pose()
            pose_prepare.position.x = mid_coordinate[0]
            pose_prepare.position.y = mid_coordinate[1]
            pose_prepare.position.z = mid_coordinate[2] + 0.05
            pose_prepare.orientation.x = 0.9999995231628418
            pose_prepare.orientation.y = 7.932441803859547e-06
            pose_prepare.orientation.z = 0.001016218215227127
            pose_prepare.orientation.w = 9.80220761448436e-07

            self.get_logger().info(f"############# Move to pose {pose_prepare} ###############")


            for _ in range(2):
                # move arm
                plan_result = self.mover.plan_pose(pick_arm, pose_prepare, planner=Planner.ompl,tip_link=_ARM_TIP[pick_arm])

                if plan_result is None:
                    self.get_logger().error(f"################### Moveit {pick_arm} Planning Failed #################")
                    if pick_arm == "Arm1":
                        pick_arm = "Arm2"
                    else:
                        pick_arm = "Arm1"
                else:
                    break

            if plan_result is None:
                continue
            else:
                break

        self.mover.execute(plan_result.trajectory)

        time.sleep(0.5)

        """
        Step 2: Any Grasp
        """
        # 夹爪保持张开, 不做闭合/提升。
        grasp_success = False
        while rclpy.ok() and not grasp_success:
            # ── 2a. GroundingDINO + SAM2 识别物体 ──────────────
            image = self.get_latest_color(_ARM_CAM[pick_arm])
            if image is None:
                self.get_logger().error("No top camera image available.")
                time.sleep(0.05)
                continue

            result = self.dino.detect(image, _GRASP_PROMPT)
            if len(result.boxes) == 0:
                self.get_logger().warn(
                    f"No '{_GRASP_PROMPT}' detected for grasp."
                )
                time.sleep(0.05)
                continue

            try:
                annotated_image = self.dino.annotate(
                    image, result, draw_mask=True
                )
            except Exception:
                annotated_image = image
            cv2.imshow("Detection Result", annotated_image)
            cv2.waitKey(1)

            # ── 2b. 驱动发布的有序彩色点云 → 物体点子集 ────────
            with self._snapshots_lock:
                cloud_msg = self._snapshots.get(
                    _ARM_CAM[pick_arm], {}
                ).get("points")

            if cloud_msg is None:
                self.get_logger().warn(
                    "No top camera point cloud available."
                )
                time.sleep(0.05)
                continue

            try:
                xyz, rgb, H, W = self._ros_pointcloud_to_organized(
                    cloud_msg
                )
            except Exception as exc:
                self.get_logger().warn(f"Point cloud parse failed: {exc}")
                time.sleep(0.05)
                continue

            mask2d = self._select_mask(result, H, W)
            if mask2d is None:
                continue

            mask2d = self._dilate_mask_3d(
                mask2d, xyz, self._mask_dilation_m
            )

            valid = np.isfinite(xyz).all(axis=2) & (xyz[..., 2] > 0)
            sel = mask2d & valid
            pts = xyz[sel]
            cols = rgb[sel]
            if len(pts) < _GRASP_MIN_POINTS:
                self.get_logger().warn(
                    f"Object point cloud too small: {len(pts)} pts."
                )
                continue

            cloud_o3d = o3d.geometry.PointCloud()
            cloud_o3d.points = o3d.utility.Vector3dVector(pts)
            cloud_o3d.colors = o3d.utility.Vector3dVector(cols)

            # ── 2c. GraspNet 抓取检测 ──────────────────────────
            gg = self._predict_grasps(cloud_o3d)
            if gg is None or len(gg) == 0:
                self.get_logger().warn("GraspNet returned no grasps.")
                continue

            # ── 2d. 过滤/排序 → 取前 10 → TF 变换到 base_link ──
            # 直接以相机 color optical frame 作为 TF 源, 用点云时间戳对齐
            # (eye-in-hand 相机随手臂移动, 必须用点云采集时刻的 TF)。
            camera_name = _ARM_CAM[pick_arm]
            optical_frame = _CAMERA_OPTICAL_FRAMES[camera_name]
            grasp_poses, gripper_geos, grasp_geos_aligned = (
                self._grasp_group_to_base_poses(
                    gg, optical_frame, cloud_msg.header.stamp
                )
            )
            if not grasp_poses:
                self.get_logger().warn(
                    "No usable grasp poses after filter/TF."
                )
                continue

            self.get_logger().info(
                f"Got {len(grasp_poses)} grasp candidates; "
                "trying best-score first."
            )

            # 在持久 Open3D 窗口显示物体点云 + top-10 夹爪图标 (复用, 不重复开窗)
            self._show_grasps_o3d(cloud_o3d, gripper_geos)

            # ── 2e. 从 score 最高开始逐个 plan, 失败换下一个 ──
            for idx, (grasp_pose, score) in enumerate(grasp_poses):
                self._publish_detection_marker(
                    (grasp_pose.position.x, grasp_pose.position.y,
                     grasp_pose.position.z)
                )
                self.get_logger().info(
                    f"Trying grasp candidate {idx + 1}/{len(grasp_poses)} "
                    f"(score={score:.3f}) for {pick_arm} ..."
                )
                plan_grasp = self.mover.plan_pose(
                    pick_arm, grasp_pose, planner=Planner.pilz_ptp,
                    tip_link=_ARM_TIP[pick_arm],
                )
                if plan_grasp is None:
                    self.get_logger().warn(
                        f"Candidate {idx + 1} plan failed; trying next."
                    )
                    continue

                if not self.mover.execute(plan_grasp.trajectory):
                    self.get_logger().warn(
                        f"Candidate {idx + 1} execute failed; trying next."
                    )
                    continue

                grasp_success = True
                self.get_logger().info(
                    f"Step 2 completed: grasped with {pick_arm} "
                    f"(candidate {idx + 1}, score={score:.3f})."
                )
                self._show_success_grasp_o3d(
                    cloud_o3d, grasp_geos_aligned[idx], score
                )
                break

            if not grasp_success:
                self.get_logger().warn(
                    "All grasp candidates failed to plan/execute; "
                    "re-detecting."
                )

            # ── 2e. Gripper close ──
            self.gripper.close(_GRIPPER[pick_arm])


    # ------------------------------------------------------------------
    # Step 2 Any Grasp: 点云解析 + GraspNet 相关
    # ------------------------------------------------------------------
    @staticmethod
    def _ros_pointcloud_to_organized(msg: PointCloud2):
        """解析驱动发布的有序彩色点云 → (xyz(H,W,3), rgb(H,W,3), H, W)。

        驱动已排除无效点 (allow_no_texture_points=false) 并完成校正,
        因此直接取 mask 对应网格即可得到物体 3D 点子集。
        """
        field_names = [f.name for f in msg.fields]
        if not all(k in field_names for k in ("x", "y", "z")):
            raise ValueError(
                f"PointCloud2 missing x/y/z fields, got {field_names}"
            )

        H, W = msg.height, msg.width
        if H <= 1:
            raise ValueError(
                f"Point cloud is not organized (height={H}); "
                "cannot apply 2D segmentation mask."
            )

        point_step = msg.point_step
        n = H * W
        raw = np.frombuffer(msg.data, dtype=np.uint8)[
            : n * point_step
        ].reshape(n, point_step)

        offsets = {
            f.name: (f.offset, _DTYPE_MAP.get(f.datatype, np.float32))
            for f in msg.fields
        }

        def get_field(name):
            off, dt = offsets[name]
            sz = np.dtype(dt).itemsize
            return np.frombuffer(
                raw[:, off: off + sz].tobytes(), dtype=dt
            ).astype(np.float32)

        x = get_field("x").reshape(H, W)
        y = get_field("y").reshape(H, W)
        z = get_field("z").reshape(H, W)
        xyz = np.stack([x, y, z], axis=2)  # (H, W, 3)

        rgb = np.zeros((H, W, 3), dtype=np.float32)
        rgb_field = (
            "rgb" if "rgb" in field_names
            else ("rgba" if "rgba" in field_names else None)
        )
        if rgb_field is not None:
            off, dt = offsets[rgb_field]
            sz = np.dtype(dt).itemsize
            rgb_raw = raw[:, off: off + sz].tobytes()
            if dt == np.float32:
                packed = np.frombuffer(
                    rgb_raw, dtype=np.float32
                ).view(np.uint32)
            else:
                packed = np.frombuffer(
                    rgb_raw, dtype=dt
                ).astype(np.uint32)

            r = ((packed >> 16) & 0xFF).astype(np.float32) / 255.0
            g = ((packed >> 8) & 0xFF).astype(np.float32) / 255.0
            b = (packed & 0xFF).astype(np.float32) / 255.0
            rgb = np.stack([r, g, b], axis=1).reshape(H, W, 3)

        return xyz, rgb, H, W

    def _select_mask(self, result, H_pc: int, W_pc: int):
        """所有检测 mask 并集 → (H_pc, W_pc) bool。无检测返回 None。"""
        if len(result.boxes) == 0:
            return None

        masks = result.masks  # (N, H_img, W_img) bool
        H_img, W_img = masks.shape[1], masks.shape[2]

        # 分辨率不一致时按最近邻缩放 mask 到点云尺寸
        if (H_img, W_img) != (H_pc, W_pc):
            resized = np.zeros(
                (masks.shape[0], H_pc, W_pc), dtype=bool
            )
            for i in range(masks.shape[0]):
                resized[i] = cv2.resize(
                    masks[i].astype(np.uint8),
                    (W_pc, H_pc),
                    interpolation=cv2.INTER_NEAREST,
                ).astype(bool)
            masks = resized

        return np.any(masks, axis=0)

    @staticmethod
    def _dilate_mask_3d(mask2d, xyz, radius_m: float):
        """3D 空间膨胀 mask: 保留 mask 点云周围 radius_m (米) 内的所有点云点。

        SAM 边缘常略紧 (少保留了一些物体表面点), 用该参数把分割点云
        往外多保留一圈。radius_m<=0 时直接返回原 mask。

        优化: 先用 cv2.dilate 在 2D 图像域把 mask 扩 N 像素 (按深度+焦距
        自适应估算 N) 作为候选区, 再用 cKDTree 仅对候选点做精确 3D 距离
        检查。旧实现对整张点云所有 valid 点 (~50w @ 1280x720) 调 query,
        单次耗时 2-5s; 优化后 <100ms, 结果完全一致。
        """
        if radius_m <= 0:
            return mask2d

        valid = np.isfinite(xyz).all(axis=2) & (xyz[..., 2] > 0)
        mask_valid = mask2d & valid
        if not np.any(mask_valid):
            return mask2d

        # 自适应估算像素半径: radius_m / depth * focal_pixel
        # focal 用 image_width/2 近似 (D4xx color 比较接近)
        mask_pts = xyz[mask_valid].reshape(-1, 3)
        median_depth = float(np.median(mask_pts[:, 2]))
        focal_guess = max(xyz.shape[1], xyz.shape[0]) * 0.5
        pixel_radius = int(np.ceil(radius_m / median_depth * focal_guess)) + 1
        pixel_radius = max(3, min(pixel_radius, 50))  # 上限 50 像素防极端值
        kernel_size = pixel_radius * 2 + 1

        kernel = np.ones((kernel_size, kernel_size), np.uint8)
        dilated_pix = cv2.dilate(
            mask_valid.astype(np.uint8), kernel, iterations=1
        ).astype(bool)
        candidates = dilated_pix & ~mask_valid & valid
        if not np.any(candidates):
            return mask2d

        # 仅对候选点做精确 3D 距离检查
        from scipy.spatial import cKDTree
        tree = cKDTree(mask_pts)
        cand_pts = xyz[candidates].reshape(-1, 3)
        dists, _ = tree.query(cand_pts, k=1)

        dilated = mask2d.copy()
        cand_flat_idx = np.where(candidates.reshape(-1))[0]
        dilated.reshape(-1)[cand_flat_idx[dists <= radius_m]] = True
        return dilated

    def _load_graspnet(self, checkpoint_path: str) -> GraspNet:
        """加载 GraspNet 抓取检测模型。"""
        net = GraspNet(
            input_feature_dim=0,
            num_view=_GRASP_NUM_VIEW,
            num_angle=12,
            num_depth=4,
            cylinder_radius=0.05,
            hmin=-0.02,
            hmax_list=[0.01, 0.02, 0.03, 0.04],
            is_training=False,
        )
        device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
        net.to(device)
        checkpoint = torch.load(checkpoint_path, map_location=device)
        net.load_state_dict(checkpoint["model_state_dict"])
        start_epoch = checkpoint.get("epoch", "?")
        self.get_logger().info(f"Loaded checkpoint (epoch: {start_epoch})")
        net.eval()
        return net

    def _predict_grasps(self, cloud_o3d: o3d.geometry.PointCloud):
        """从物体点云预测抓取, 返回 GraspGroup 或 None。"""
        points = np.asarray(cloud_o3d.points, dtype=np.float32)
        colors = np.asarray(cloud_o3d.colors, dtype=np.float32)
        if len(points) == 0:
            return None

        # ── 降采样到 num_point ──────────────────────────────
        if len(points) >= _GRASP_NUM_POINT:
            idxs = np.random.choice(
                len(points), _GRASP_NUM_POINT, replace=False
            )
        else:
            idxs1 = np.arange(len(points))
            idxs2 = np.random.choice(
                len(points), _GRASP_NUM_POINT - len(points), replace=True
            )
            idxs = np.concatenate([idxs1, idxs2], axis=0)

        cloud_sampled = points[idxs]
        color_sampled = (
            colors[idxs]
            if len(colors) == len(points)
            else np.zeros_like(points)
        )

        device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
        cloud_t = torch.from_numpy(
            cloud_sampled[np.newaxis].astype(np.float32)
        ).to(device)

        end_points = {
            "point_clouds": cloud_t,
            "cloud_colors": color_sampled,
        }

        with torch.no_grad():
            end_points = self._graspnet(end_points)
            grasp_preds = pred_decode(end_points)

        gg_array = grasp_preds[0].detach().cpu().numpy()
        return GraspGroup(gg_array)

    def _grasp_group_to_base_poses(self, gg, frame_id: str, stamp):
        """过滤/排序 GraspGroup → 取前 10 → 轴重映射 → TF 变换到 base_link。

        GraspNet 姿态 (位于点云所在光学系 color_optical_frame) 先做夹爪
        坐标系→机械臂 tip frame 的轴重映射 (R_tool = R_grasp @ _GRASP_TO_TOOL,
        使机械臂接近轴 +Z 对齐 graspnet 接近方向 +X), 再经 TF 变换到 base_link。

        Parameters
        ----------
        gg : GraspGroup
            原始抓取集合 (位于 frame_id 坐标系).
        frame_id : str
            TF 源 frame (相机 color optical frame).
        stamp : rclpy time
            点云时间戳, 用于 eye-in-hand 相机的 TF 插值 (与点云采集时刻对齐).

        Returns
        -------
        tuple[list[tuple[Pose, float]], list, list]
            (base_link 下的抓取 pose, score) 按 score 降序,
            以及夹爪几何体 (相机坐标系, 供 Open3D 显示),
            以及与 poses 一一对齐的夹爪几何体 (TF 失败被跳过的候选无对应项)。
        """
        # 排除夹爪长度(depth) > 9.5cm 的抓取
        if _GRASP_MAX_DEPTH > 0 and len(gg) > 0:
            keep = np.where(gg.depths <= _GRASP_MAX_DEPTH)[0]
            gg = gg[keep]
        if len(gg) == 0:
            return [], [], []

        gg.nms()
        gg.sort_by_score()
        gg = gg[: _GRASP_TOP_K]

        gripper_geos = gg.to_open3d_geometry_list()

        poses = []
        geos_aligned = []
        for i, g in enumerate(gg):
            tool_R = g.rotation_matrix @ _GRASP_TO_TOOL
            quat = R.from_matrix(tool_R).as_quat()
            pose = Pose()
            pose.position.x = float(g.translation[0])
            pose.position.y = float(g.translation[1])
            pose.position.z = float(g.translation[2])
            pose.orientation.x = quat[0]
            pose.orientation.y = quat[1]
            pose.orientation.z = quat[2]
            pose.orientation.w = quat[3]

            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = frame_id
            pose_stamped.header.stamp = stamp
            pose_stamped.pose = pose

            try:
                transformed = self._tf_buffer.transform(
                    pose_stamped, "base_link",
                    timeout=rclpy.duration.Duration(seconds=0.5),
                )
            except (tf2_ros.LookupException,
                    tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException) as e:
                self.get_logger().warn(
                    f"[{frame_id}] grasp tf transform failed: {e}",
                    throttle_duration_sec=2.0,
                )
                continue

            poses.append((transformed.pose, g.score))
            geos_aligned.append(gripper_geos[i])

        return poses, gripper_geos, geos_aligned

    def _show_grasps_o3d(
        self,
        cloud_o3d: o3d.geometry.PointCloud,
        gripper_geos,
    ) -> None:
        """持久单窗口显示物体点云 + GraspNet 夹爪图标 (非阻塞)。

        只创建/保留一个 Open3D 窗口 (daemon 线程)。每次有新点云数据到来时
        清空上一帧几何体, 在原窗口重新显示, 不重复创建窗口、不画坐标系。
        按 Esc/Q 或关闭窗口可退出 (退出后再调用会重新开窗)。
        """
        with self._vis_lock:
            self._vis_pending = (cloud_o3d, list(gripper_geos))
            if (
                self._vis_thread is None
                or not self._vis_thread.is_alive()
            ):
                self._vis_thread = threading.Thread(
                    target=self._vis_loop, daemon=True
                )
                self._vis_thread.start()

    def _vis_loop(self) -> None:
        """Open3D 可视化主循环 (常驻, 消费 _vis_pending 更新同一窗口)。"""
        try:
            vis = o3d.visualization.VisualizerWithKeyCallback()
            vis.create_window(
                window_name="GraspNet Grasps - Segmented Object",
                width=1280,
                height=720,
            )
            shown_geos = []
            running = True

            def _on_quit(_vis):
                nonlocal running
                running = False
                return False

            vis.register_key_callback(27, _on_quit)       # Esc
            vis.register_key_callback(ord("Q"), _on_quit)  # Q

            vis.add_geometry(
                o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)
            )

            while running and rclpy.ok():
                with self._vis_lock:
                    pending = self._vis_pending
                    self._vis_pending = None

                if pending is not None:
                    # 清空上一帧, 在同一窗口显示新数据
                    for geo in shown_geos:
                        vis.remove_geometry(geo, reset_bounding_box=False)
                    shown_geos = []

                    cloud_o3d, grippers = pending
                    vis.add_geometry(cloud_o3d, reset_bounding_box=True)
                    shown_geos.append(cloud_o3d)
                    for g in grippers:
                        vis.add_geometry(g, reset_bounding_box=False)
                        shown_geos.append(g)

                if not vis.poll_events():
                    break
                vis.update_renderer()
                time.sleep(0.01)

            vis.destroy_window()
        except Exception as exc:
            self.get_logger().warn(
                f"Open3D grasp display failed: {exc}"
            )
        finally:
            with self._vis_lock:
                if self._vis_thread == threading.current_thread():
                    self._vis_thread = None

    def _show_success_grasp_o3d(
        self,
        cloud_o3d: o3d.geometry.PointCloud,
        gripper_geo,
        score: float,
    ) -> None:
        """在【新】Open3D 窗口显示真正规划/执行成功的夹爪 (非阻塞, 独立窗口)。

        与 _show_grasps_o3d 的常驻单窗口不同, 该方法每次被调用都新开一个
        窗口, 只显示物体点云 + 该次成功的那个夹爪 (不再画出全部 top-10)。
        点云与夹爪几何均位于相机 optical frame, 坐标系一致可直接叠加。
        按 Esc/Q 或关闭窗口退出。
        """
        def _success_loop():
            try:
                vis = o3d.visualization.VisualizerWithKeyCallback()
                vis.create_window(
                    window_name=f"Success Grasp - score={score:.3f}",
                    width=1280,
                    height=720,
                )
                running = True

                def _on_quit(_vis):
                    nonlocal running
                    running = False
                    return False

                vis.register_key_callback(27, _on_quit)       # Esc
                vis.register_key_callback(ord("Q"), _on_quit)  # Q

                vis.add_geometry(cloud_o3d, reset_bounding_box=True)
                if gripper_geo is not None:
                    vis.add_geometry(gripper_geo,
                                     reset_bounding_box=False)

                while running and rclpy.ok():
                    if not vis.poll_events():
                        break
                    vis.update_renderer()
                    time.sleep(0.01)

                vis.destroy_window()
            except Exception as exc:
                self.get_logger().warn(
                    f"Open3D success-grasp display failed: {exc}"
                )

        threading.Thread(target=_success_loop, daemon=True).start()

    # ------------------------------------------------------------------
    # 检测点 Marker 可视化
    # ------------------------------------------------------------------
    def _publish_detection_marker(
        self,
        coordinate: Tuple[float, float, float],
    ) -> None:
        """在 RViz 中发布一个红色小球 Marker 标出检测到的 3D 点。"""
        marker = Marker()
        marker.header.frame_id = 'base_link'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'detection'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = coordinate[0]
        marker.pose.position.y = coordinate[1]
        marker.pose.position.z = coordinate[2]
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.03
        marker.scale.y = 0.03
        marker.scale.z = 0.03
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.lifetime.sec = 0  # 0 = forever
        self._marker_pub.publish(marker)
        self.get_logger().info(
            f"Detection marker published at "
            f"({coordinate[0]:.4f}, {coordinate[1]:.4f}, {coordinate[2]:.4f})"
        )

    # ------------------------------------------------------------------
    # 彩色回调
    # ------------------------------------------------------------------
    def _color_callback(self, camera_name: str, msg: Image) -> None:
        """将彩色帧转为 BGR 并存入快照。"""
        try:
            color_bgr = self.ros_image_to_cv2(msg)
        except Exception as e:
            self.get_logger().error(
                f"[{camera_name}] color conversion failed: {e}",
                throttle_duration_sec=5.0,
            )
            return
        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        with self._snapshots_lock:
            snap = self._snapshots.setdefault(camera_name, {})
            snap['color'] = color_bgr
            snap['stamp_ns'] = stamp_ns

    # ------------------------------------------------------------------
    # 点云回调
    # ------------------------------------------------------------------
    def _cloud_callback(self, camera_name: str, msg: PointCloud2) -> None:
        """缓存最新一帧驱动发布的有序彩色点云 (只存 msg 引用)。"""
        with self._snapshots_lock:
            snap = self._snapshots.setdefault(camera_name, {})
            snap['points'] = msg

    # ------------------------------------------------------------------
    # 彩色查询 API
    # ------------------------------------------------------------------
    def get_latest_color(
        self,
        camera_name: str,
        min_stamp: Optional[rclpy.time.Time] = None,
    ) -> Optional[np.ndarray]:
        """返回最近一帧彩色图 (H×W×3 BGR uint8).

        Parameters
        ----------
        camera_name : str
            相机名.
        min_stamp : rclpy.time.Time or None
            若提供，则只返回时间戳晚于此值的帧，否则返回 None（用于等待新帧）。

        Returns
        -------
        np.ndarray or None
            无快照或帧过旧时返回 None.
        """
        with self._snapshots_lock:
            snap = self._snapshots.get(camera_name)
            if snap is None or 'color' not in snap:
                return None
            if min_stamp is not None:
                min_ns = min_stamp.nanoseconds
                if snap.get('stamp_ns', 0) < min_ns:
                    return None
            return snap['color']

    # ------------------------------------------------------------------
    # 深度回调
    # ------------------------------------------------------------------
    def _depth_callback(self, camera_name: str, msg: Image) -> None:
        """将对齐深度帧存入快照。"""
        try:
            depth = self._ros_depth_to_mm(msg)
        except Exception as e:
            self.get_logger().error(
                f"[{camera_name}] depth conversion failed: {e}",
                throttle_duration_sec=5.0,
            )
            return
        with self._snapshots_lock:
            snap = self._snapshots.setdefault(camera_name, {})
            snap['depth'] = depth

    # ------------------------------------------------------------------
    # 深度查询 API
    # ------------------------------------------------------------------
    def get_depth_at_pixel(
        self,
        camera_name: str,
        u: int,
        v: int,
        window: int = 1,
    ) -> Optional[float]:
        """查询指定像素位置的最新深度值 (物理尺度, 毫米).

        Parameters
        ----------
        camera_name : str
            相机名, 如 ``"camera_top"``, ``"camera_left"``, ``"camera_right"``.
        u, v : int
            像素坐标 (列, 行), 原点为图像左上角.
        window : int
            取该窗口内所有有效深度的中位数, 默认为 1 (即单像素).
            增大窗口可提高抗噪能力, 奇数.

        Returns
        -------
        float or None
            深度值 (mm), 如果无最新帧或无有效值则返回 None.
        """
        u = int(u)
        v = int(v)

        with self._snapshots_lock:
            snap = self._snapshots.get(camera_name)
            depth = snap.get('depth') if snap else None

        if depth is None:
            self.get_logger().warn(
                f"[{camera_name}] no depth frame available",
                throttle_duration_sec=2.0,
            )
            return None

        h, w = depth.shape
        half = window // 2

        # 边界裁剪
        r_min = max(0, v - half)
        r_max = min(h, v + half + 1)
        c_min = max(0, u - half)
        c_max = min(w, u + half + 1)

        patch = depth[r_min:r_max, c_min:c_max]
        valid = patch[patch > 0]  # 0 = 无效值

        if valid.size == 0:
            return None

        return float(np.median(valid))

    def get_latest_depth(self, camera_name: str) -> Optional[np.ndarray]:
        """返回最近一帧对齐深度图 (mm, uint16), 无最新帧返回 None."""
        with self._snapshots_lock:
            snap = self._snapshots.get(camera_name)
            return snap.get('depth') if snap else None

    # ------------------------------------------------------------------
    # 相机内参回调
    # ------------------------------------------------------------------
    def _camera_info_callback(self, camera_name: str, msg: CameraInfo) -> None:
        """存入 camera_info 快照 (用于 pixel→3D 反投影)."""
        with self._camera_info_lock:
            self._camera_infos[camera_name] = msg

    # ------------------------------------------------------------------
    # 像素 → base_link 3D 坐标转换
    # ------------------------------------------------------------------
    def pixel_to_base_link(
        self,
        camera_name: str,
        u: int,
        v: int,
        window: int = 1,
        timeout: float = 0.1,
    ) -> Optional[Tuple[float, float, float]]:
        """将深度图上的像素点转换为 ``base_link`` 下的 3D 坐标。

        流程:
          1. 从缓存的深度图中取像素 (u, v) 的深度 (mm)
          2. 用相机内参反投影到相机光学坐标系 (m)
          3. 通过 tf2 变换到 ``base_link``

        Parameters
        ----------
        camera_name : str
            ``"camera_top"`` / ``"camera_left"`` / ``"camera_right"``.
        u, v : int
            像素坐标 (列, 行).
        window : int
            深度查询的窗口大小, 见 ``get_depth_at_pixel``.
        timeout : float
            tf2 lookup 超时 (秒).

        Returns
        -------
        tuple (x, y, z) or None
            ``base_link`` 下的 3D 坐标 (m), 任一环节失败返回 None.
        """
        # ── 1. 获取深度 ──────────────────────────────────────
        depth_mm = self.get_depth_at_pixel(camera_name, u, v, window=window)
        if depth_mm is None:
            return None
        depth_m = depth_mm / 1000.0

        # ── 2. 获取相机内参 ──────────────────────────────────
        with self._camera_info_lock:
            info = self._camera_infos.get(camera_name)
        if info is None:
            self.get_logger().warn(
                f"[{camera_name}] no camera_info available",
                throttle_duration_sec=2.0,
            )
            return None

        # ── 3. 反投影: 像素 → 相机光学坐标 (右手系: X右 Y下 Z前) ──
        fx, fy = info.k[0], info.k[4]
        cx, cy = info.k[2], info.k[5]

        x_cam = (u - cx) / fx * depth_m
        y_cam = (v - cy) / fy * depth_m
        z_cam = depth_m

        # ── 4. tf2 变换到 base_link ──────────────────────────
        optical_frame = _CAMERA_OPTICAL_FRAMES.get(camera_name)
        if optical_frame is None:
            self.get_logger().error(
                f"[{camera_name}] unknown optical frame — "
                f"update _CAMERA_OPTICAL_FRAMES",
                throttle_duration_sec=10.0,
            )
            return None

        p_cam = PointStamped()
        p_cam.header.frame_id = optical_frame
        p_cam.header.stamp = self.get_clock().now().to_msg()
        p_cam.point.x = x_cam
        p_cam.point.y = y_cam
        p_cam.point.z = z_cam

        try:
            p_base = self._tf_buffer.transform(
                p_cam, "base_link",
                timeout=rclpy.duration.Duration(seconds=timeout),
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(
                f"[{camera_name}] tf lookup failed: {e}",
                throttle_duration_sec=2.0,
            )
            return None

        return (p_base.point.x, p_base.point.y, p_base.point.z)

    # ------------------------------------------------------------------
    # 图像转换工具
    # ------------------------------------------------------------------
    @staticmethod
    def _ros_depth_to_mm(msg: Image) -> np.ndarray:
        """将 ``sensor_msgs/Image`` (16UC1) 转为 uint16 深度图 (mm)。"""
        if msg.encoding not in _DEPTH_ENCODINGS:
            raise ValueError(
                f"Unsupported depth encoding: '{msg.encoding}', "
                f"expected one of {_DEPTH_ENCODINGS}"
            )
        expected_step = msg.width * 2
        if msg.step != expected_step:
            raise ValueError(
                f"Unexpected step={msg.step}, expected {expected_step}"
            )
        raw = np.frombuffer(msg.data, dtype=np.uint16)
        return raw.reshape((msg.height, msg.width))

    @staticmethod
    def ros_image_to_cv2(msg: Image) -> np.ndarray:
        """将 ``sensor_msgs/Image`` 转换为 OpenCV BGR 图像 (不依赖 cv_bridge)。

        支持 ``bgr8`` 和 ``rgb8`` 编码。

        Parameters
        ----------
        msg : Image
            ROS 图像消息

        Returns
        -------
        np.ndarray
            H×W×3 uint8 BGR 图像
        """
        if msg.encoding not in _COLOR_ENCODINGS:
            raise ValueError(
                f"Unsupported encoding: '{msg.encoding}', "
                f"expected one of {_COLOR_ENCODINGS}"
            )
        expected_step = msg.width * 3
        if msg.step != expected_step:
            raise ValueError(
                f"Unexpected step={msg.step}, expected {expected_step}"
            )

        raw = np.frombuffer(msg.data, dtype=np.uint8)
        frame = raw.reshape((msg.height, msg.width, 3))
        if msg.encoding == "rgb8":
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        return frame

    # ------------------------------------------------------------------
    # 位姿读取
    # ------------------------------------------------------------------
    def read_current_poses(self) -> Dict[str, Pose]:
        """读取双臂当前位姿, 返回 ``{"Arm1": Pose, "Arm2": Pose}``。"""
        poses: Dict[str, Pose] = {}
        for arm in ("Arm1", "Arm2"):
            try:
                pose = self.mover.get_current_pose(arm)
                p = pose.position
                q = pose.orientation
                self.get_logger().info(
                    f"[{arm}] current pose — "
                    f"pos: ({p.x:.4f}, {p.y:.4f}, {p.z:.4f}), "
                    f"ori: ({q.x:.4f}, {q.y:.4f}, {q.z:.4f}, {q.w:.4f})",
                )
                poses[arm] = pose
            except Exception as e:
                self.get_logger().error(f"[{arm}] failed to read pose: {e}")
                poses[arm] = Pose()
        return poses


def main():
    rclpy.init(args=sys.argv)

    node = XTrainerTask()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()