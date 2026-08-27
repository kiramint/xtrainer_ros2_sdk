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
import traceback
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
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sam2.automatic_mask_generator import SAM2AutomaticMaskGenerator
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from visualization_msgs.msg import Marker

from graspnet.collision_detector import ModelFreeCollisionDetector
from graspnet.graspnet import GraspNet, pred_decode
from xtrainer_control.robot_control import RobotController
from xtrainer_gripper.gripper_control import GripperController
from xtrainer_task.dino_wrapper import DinoWrapper
from xtrainer_task.robot_move import Planner, RobotMover

_COLOR_ENCODINGS = {"bgr8", "rgb8"}
_DEPTH_ENCODINGS = {"16UC1", "mono16"}

# 传感器数据 QoS: 只保留最新 1 帧 + 尽力传输。
# depth=10 Reliable 时, SAM/GraspNet 推理期间回调若被 GIL 挤压, 最多积压
# 10 帧 (~0.67s @15fps), 推理结束后按旧→新逐帧处理, 表现为陈旧帧延迟;
# depth=1 + BEST_EFFORT 让执行器永远取最新帧 (与 realsense 的 Reliable
# 发布端 QoS 兼容: pub RELIABLE + sub BEST_EFFORT 可匹配)。
_SENSOR_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT,
)

# 彩色 / 对齐深度话题模板
_COLOR_TOPIC_TEMPLATE = "/camera/{name}/color/image_raw"
_DEPTH_TOPIC_TEMPLATE = "/camera/{name}/aligned_depth_to_color/image_raw"

# 只使用顶部 D435 相机 (固定安装, eye-on-base), 不订阅左右手腕相机
_CAMERA_NAMES = ("camera_top_435",)

_CAMERA_OPTICAL_FRAMES: Dict[str, str] = {
    "camera_top_435": "camera_top_435_color_optical_frame",
    "camera_top": "camera_top_color_optical_frame",
    "camera_left": "camera_left_color_optical_frame",
    "camera_right": "camera_right_color_optical_frame",
}
_ARM_TIP = {
    "Arm1": "L1_gripper_tip",
    "Arm2": "L2_gripper_tip",
}
_ARM_CAM = {
    "Arm1": "camera_top_435",
    "Arm2": "camera_top_435",
}
_GRIPPER = {
    "Arm1": "left",
    "Arm2": "right",
}

# ── Step 2 Any Grasp: SAM automask 物体并集 → GraspNet → mask 过滤 ────
_GRASP_PC_TOPIC_TEMPLATE = "/camera/{name}/depth/color/points"
_GRASP_NUM_POINT = 20000
_GRASP_NUM_VIEW = 300
# 夹爪长度(depth)上限, 单位 m, 过滤 depth>此值的抓取。9.5cm=0.095
_GRASP_MAX_DEPTH = 0.095
_GRASP_TOP_K = 10
_GRASP_ANGLE_LIMIT = 25  # 接近方向与世界Z轴的最大夹角(度), 0=不过滤
_GRASP_MIN_POINTS = 50
# Model-free collision filter (GraspNet scene IoU against object cloud)
_GRASP_COLLISION_VOXEL_SIZE = 0.005
_GRASP_COLLISION_APPROACH_DIST = 0.03
_GRASP_COLLISION_THRESH = 0.05

# Pre-grasp 后撤距离 (cm): 抓取前夹爪先运动到 grasp 姿态沿局部 -Z 轴
# (接近方向反方向, 远离物体) 后撤此距离的预备点, 再直线进给到 grasp 点。
# 调整后撤量改这一个变量即可。
_GRASP_PRE_GRASP_OFFSET_CM = 10.0
_GRASP_PRE_GRASP_OFFSET_M = _GRASP_PRE_GRASP_OFFSET_CM / 100.0

_GRASP_Z_LIMIT = 0.0461

# ── Desk Detect: GroundingDINO + SAM2 白色桌面 → ROI → 自动掩码物体分割 ─
_DESK_CAMERA = "camera_top_435"
_DESK_PROMPT = "white square."
_DESK_ROI_SHRINK_PX = 0           # ROI 整体向内缩 x 像素 (正值缩小)
_DESK_MIN_AREA_RATIO = 0.002      # 自动掩码面积下限 (相对 ROI 面积比例)
_DESK_DROP_LARGEST_MASK = True    # 排除面积最大的 mask (桌子表面本身)
_DESK_WINDOW_NAME = "Desk Object Detect"

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
        self.declare_parameter("mask_dilation_m", 0.8)
        self._mask_dilation_m = float(
            self.get_parameter("mask_dilation_m").value
        )

        # MoveItPy reads configs from the parameter server (set by launch file)
        self._moveit = MoveItPy(node_name='xtrainer_task_moveit')
        self.mover = RobotMover(self._moveit, self)

        # 机械臂状态控制 (独立 callback group, 不与 MoveIt/timer 竞争)
        self._robot_ctrl = RobotController(self)

        # Give MoveIt some time to receive latest /joint_states and TF
        self.get_logger().info('Waiting for MoveIt state to populate …')
        time.sleep(2.0)

        # DINO 检测器 (SAM2 用 base_plus, 与 start_grasp_plane.py 一致)
        self.dino = DinoWrapper(
            device='cuda', box_threshold=0.35, text_threshold=0.25,
            sam2_checkpoint="/opt/Project/Grounded-SAM-2/checkpoints/sam2.1_hiera_base_plus.pt",
            sam2_config="configs/sam2.1/sam2.1_hiera_b+.yaml",
        )

        # SAM2 自动掩码生成器 (复用 DinoWrapper 加载的 sam2_model 权重)
        # 用于在白色桌面 ROI 内自动切分桌上各个物体
        # 参数与 start_grasp_plane.py 一致 (base_plus + 32 点 + 单层裁剪,
        # 实测该组参数对桌上物品分割效果最好)
        self._amg = SAM2AutomaticMaskGenerator(
            model=self.dino.sam2_model,
            points_per_side=32,
            pred_iou_thresh=0.85,
            stability_score_thresh=0.90,
            min_mask_region_area=0,
            crop_n_layers=0,
            box_nms_thresh=0.7,
            output_mode="binary_mask",
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

        # ── 桌面物体检测最新结果 (launch 循环每帧刷新, 供外部读取) ──
        self._latest_desk_objects = None

        # ARM
        self.pick_arm = "Arm1"

        # ── TF2 缓冲与监听器 ──
        # cache_time 调大到 120s: Step2 的 SAM2+GraspNet 单轮推理 ~15s,
        # 若 TF 失败还会进入 while 循环重试, 累积可达数十秒. 默认 10s cache
        # 会在 SAM/GraspNet 完成前就把点云采集时刻的 TF 丢弃, 导致
        # "Lookup would require extrapolation into the past".
        self._tf_buffer = tf2_ros.Buffer(
            cache_time=rclpy.duration.Duration(seconds=120)
        )
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._camera_cb_group = ReentrantCallbackGroup()
        self._cloud_cb_group = MutuallyExclusiveCallbackGroup()


        # 为顶部相机 (camera_top_435) 订阅彩色、对齐深度、内参 & 点云话题
        for name in _CAMERA_NAMES:
            color_topic = _COLOR_TOPIC_TEMPLATE.format(name=name)
            self.create_subscription(
                Image, color_topic,
                lambda msg, cam=name: self._color_callback(cam, msg),
                _SENSOR_QOS,
                callback_group=self._camera_cb_group,
            )
            self.get_logger().info(f"Subscribed color: {color_topic}")

            depth_topic = _DEPTH_TOPIC_TEMPLATE.format(name=name)
            self.create_subscription(
                Image, depth_topic,
                lambda msg, cam=name: self._depth_callback(cam, msg),
                _SENSOR_QOS,
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
                _SENSOR_QOS,
                callback_group=self._cloud_cb_group,
            )
            self.get_logger().info(f"Subscribed pointcloud: {cloud_topic}")

        # ── 检测点 Marker 发布器 (RViz 可视化) ──
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._marker_pub = self.create_publisher(Marker, '/detection_marker', qos)

        self.get_logger().info('#################### All module initialized ######################')

    # ------------------------------------------------------------------
    # Controll Task ####################################################
    # ------------------------------------------------------------------

    def step_any_grasp(self, results):
        """基于顶部相机 SAM 物体分割结果提取点云 → GraspNet → 抓取。

        直接使用顶部相机 (camera_top_435) 完成识别与抓取, 不再依赖
        左右手腕相机, 也不再先移动到物体上方精拍。

        流程:
            1. 合并 step_detect_desk_objects 输出的物体 mask 并集
            2. 取 camera_top_435 有序点云, mask 3D 膨胀后提取场景点云
            3. GraspNet 检测抓取位姿
            4. 过滤: 夹取中心 (反投影像素) 必须落在 SAM 物体 mask 内
            5. 按 score 降序取 top-K, 从最高分开始逐个规划执行

        Parameters
        ----------
        results : list[dict]
            step_detect_desk_objects 的输出, 每项含 "mask" (H, W) bool。

        Returns
        -------
        bool
            是否成功抓取 (机械臂到位并闭合夹爪)。
        """
        grasp_success = False
        camera_name = _ARM_CAM[self.pick_arm]

        # ── 2a. SAM 物体 mask 并集 (图像分辨率) ──────────────
        if not results:
            self.get_logger().warn("step_any_grasp: empty object masks.")
            return False
        H_img, W_img = results[0]["mask"].shape[:2]
        mask_union = np.zeros((H_img, W_img), dtype=bool)
        for r in results:
            mask_union |= np.asarray(r["mask"], dtype=bool)

        # ── 2b. 顶部相机有序点云 → mask 并集点子集 ────────────
        with self._snapshots_lock:
            cloud_msg = self._snapshots.get(camera_name, {}).get("points")

        if cloud_msg is None:
            self.get_logger().warn(
                f"No {camera_name} point cloud available."
            )
            return False

        try:
            xyz, rgb, H, W = self._ros_pointcloud_to_organized(cloud_msg)
        except Exception as exc:
            self.get_logger().warn(f"Point cloud parse failed: {exc}")
            return False

        valid = np.isfinite(xyz).all(axis=2) & (xyz[..., 2] > 0)

        mask2d = mask_union
        if mask2d.shape != (H, W):
            mask2d = cv2.resize(
                mask2d.astype(np.uint8), (W, H),
                interpolation=cv2.INTER_NEAREST,
            ).astype(bool)

        self.get_logger().info("start dilating mask")
        mask2d = self._dilate_mask_3d(mask2d, xyz, self._mask_dilation_m)
        self.get_logger().info("complete dilating mask")

        sel = mask2d & valid
        pts = xyz[sel]
        cols = rgb[sel]
        if len(pts) < _GRASP_MIN_POINTS:
            self.get_logger().warn(
                f"Object point cloud too small: {len(pts)} pts."
            )
            return False

        mfcdetector = ModelFreeCollisionDetector(
            pts, voxel_size=_GRASP_COLLISION_VOXEL_SIZE
        )

        cloud_o3d = o3d.geometry.PointCloud()
        cloud_o3d.points = o3d.utility.Vector3dVector(pts)
        cloud_o3d.colors = o3d.utility.Vector3dVector(cols)

        # ── 2c. GraspNet 抓取检测 ──────────────────────────
        self.get_logger().info("start grasp detection")
        gg = self._predict_grasps(cloud_o3d)
        if gg is None or len(gg) == 0:
            self.get_logger().warn("GraspNet returned no grasps.")
            return False

        # ── 2c-2. ModelFreeCollisionDetector 碰撞过滤 ────────
        n_before = len(gg)
        collision_mask, iou_list = mfcdetector.detect(
            gg,
            approach_dist=_GRASP_COLLISION_APPROACH_DIST,
            collision_thresh=_GRASP_COLLISION_THRESH,
            return_ious=True,
        )
        n_collide = int(np.count_nonzero(collision_mask))
        global_iou = iou_list[0]
        self.get_logger().info(
            f"Collision filter: removed {n_collide}/{n_before} grasps "
            f"(thresh={_GRASP_COLLISION_THRESH}, "
            f"mean global IoU={float(np.mean(global_iou)):.3f})."
        )
        gg = gg[~collision_mask]
        if len(gg) == 0:
            self.get_logger().warn(
                "All grasps in collision with the object cloud."
            )
            return False

        self._show_grasps_o3d(cloud_o3d, gg.to_open3d_geometry_list())

        # ── 2d. 过滤 (夹取中心须落在 SAM mask 内) / 排序 → TF ─
        # 顶部相机固定安装 (eye-on-base), 用点云时间戳对齐 TF 即可。
        self.get_logger().info("start filter grasp pose")
        optical_frame = _CAMERA_OPTICAL_FRAMES[camera_name]
        with self._camera_info_lock:
            cam_info = self._camera_infos.get(camera_name)
        grasp_poses, gripper_geos, grasp_geos_aligned = (
            self._grasp_group_to_base_poses(
                gg, optical_frame, cloud_msg.header.stamp,
                object_mask=mask_union, camera_info=cam_info,
            )
        )
        if not grasp_poses:
            self.get_logger().warn(
                "No usable grasp poses after filter/TF."
            )
            return False

        self.get_logger().info(
            f"Got {len(grasp_poses)} grasp candidates; "
            "trying best-score first."
        )

        # 在持久 Open3D 窗口显示物体点云 + top-10 夹爪图标 (复用, 不重复开窗)
        self._show_grasps_o3d(cloud_o3d, gripper_geos)

        # ── 2e. 从 score 最高开始逐个: pre-grasp → grasp → close ──
        for idx, (grasp_pose, score) in enumerate(grasp_poses):
            current_joints = self.mover.get_current_joint_positions(
                self.pick_arm
            )
            grasp_pose = self._tip_toward_tcp_offset(grasp_pose, -0.03)
            if grasp_pose.position.z < _GRASP_Z_LIMIT:
                grasp_pose.position.z = _GRASP_Z_LIMIT
            self._publish_detection_marker(
                (grasp_pose.position.x, grasp_pose.position.y,
                 grasp_pose.position.z)
            )
            self.get_logger().info(
                f"Trying grasp candidate {idx + 1}/{len(grasp_poses)} "
                f"(score={score:.3f}) for {self.pick_arm} ..."
            )

            # Generate pre_grasp
            pre_grasp_pose = self._tip_toward_tcp_offset(
                grasp_pose, _GRASP_PRE_GRASP_OFFSET_M
            )
            p_pre = pre_grasp_pose.position
            self.get_logger().info(
                f"Pre-grasp {idx + 1}: retreat "
                f"{_GRASP_PRE_GRASP_OFFSET_CM:.1f} cm along -Z to "
                f"({p_pre.x:.4f}, {p_pre.y:.4f}, {p_pre.z:.4f})"
            )

            plan_pre = self.mover.plan_pose(
                self.pick_arm, pre_grasp_pose, planner=Planner.ompl,
                tip_link=_ARM_TIP[self.pick_arm],
            )
            if plan_pre is None:
                self.get_logger().warn(
                    f"Candidate {idx + 1} pre-grasp plan failed; "
                    "trying next."
                )
                continue

            # 大角度 J6 等价目标修正放在搬运段 (pre-grasp) 完成,
            # 之后的直线进给段姿态不再大改
            plan_pre = self._flip_j6_if_needed(
                plan_pre, current_joints, self.pick_arm, idx
            )

            if not self.mover.execute(plan_pre.trajectory):
                self.get_logger().warn(
                    f"Candidate {idx + 1} pre-grasp execute failed; "
                    "trying next."
                )
                continue

            # ── 2e-2. grasp: 从 pre-grasp 直线进给到抓取点 ──
            joints_after_pre = self.mover.get_current_joint_positions(
                self.pick_arm
            )
            plan_grasp = self.mover.plan_pose(
                self.pick_arm, grasp_pose, planner=Planner.pilz_ptp,
                tip_link=_ARM_TIP[self.pick_arm],
            )
            # if plan_grasp is None:
            #     plan_grasp = self.mover.plan_pose(
            #         self.pick_arm, grasp_pose, planner=Planner.pilz_ptp,
            #         tip_link=_ARM_TIP[self.pick_arm],
            #     )
            if plan_grasp is None:
                self.get_logger().warn(
                    f"Candidate {idx + 1} plan failed; trying next."
                )
                continue

            plan_grasp = self._flip_j6_if_needed(
                plan_grasp, joints_after_pre, self.pick_arm, idx
            )

            if not self.mover.execute(plan_grasp.trajectory):
                self.get_logger().warn(
                    f"Candidate {idx + 1} execute failed; trying next."
                )
                continue

            grasp_success = True
            self.get_logger().info(
                f"Step 2 completed: grasped with {self.pick_arm} "
                f"(candidate {idx + 1}, score={score:.3f})."
            )
            self._show_success_grasp_o3d(
                cloud_o3d, grasp_geos_aligned[idx], score
            )
            break

        if not grasp_success:
            self.get_logger().warn(
                "All grasp candidates failed to plan/execute."
            )
            return False

        # ── 2f. Gripper close ──
        self.gripper.close(_GRIPPER[self.pick_arm])
        return True

    def step_go_up(self):
        pose = Pose()
        if self.pick_arm == "Arm1":
            pose.position.x = 0.3217
            pose.position.y = -0.0975
            pose.position.z = 0.5333
            pose.orientation.x = 0.6618
            pose.orientation.y = -0.6384
            pose.orientation.z = 0.2761
            pose.orientation.w = -0.2796
        else:
            pose.position.x = 0.6788
            pose.position.y = -0.0987
            pose.position.z = 0.5367
            pose.orientation.x = 0.6287
            pose.orientation.y = 0.6488
            pose.orientation.z = -0.3000
            pose.orientation.w = -0.3063

        plan_result = self.mover.plan_pose(self.pick_arm, pose, planner=Planner.pilz_ptp, tip_link=_ARM_TIP[self.pick_arm])
        if plan_result is None:
            self.get_logger().error(f"################### Moveit {self.pick_arm} Planning Failed #################")
            return False
        self.mover.execute(plan_result.trajectory)
        return True

    def step_place_object(self):
        pose = Pose()
        if self.pick_arm == "Arm1":
            pose.position.x = 0.0677
            pose.position.y = -0.2794
            pose.position.z = 0.3030
            pose.orientation.x = 0.9992
            pose.orientation.y = -0.0370
            pose.orientation.z = 0.0067
            pose.orientation.w = 0.0109
        else:
            pose.position.x = 1.0077
            pose.position.y = -0.2945
            pose.position.z = 0.3061
            pose.orientation.x = 0.9994
            pose.orientation.y = -0.0184
            pose.orientation.z = 0.0198
            pose.orientation.w = -0.0211

        plan_result = self.mover.plan_pose(self.pick_arm, pose, planner=Planner.pilz_ptp, tip_link=_ARM_TIP[self.pick_arm])
        if plan_result is None:
            self.get_logger().error(f"################### Moveit {self.pick_arm} Planning Failed #################")
            return False
        self.mover.execute(plan_result.trajectory)
        self.gripper.open(_GRIPPER[self.pick_arm])
        return True


    def launch(self):
        self.gripper.open_both()

        try_times = 3

        while True:
            if try_times <= 0:
                break

            # Step 1: 顶部相机 (camera_top_435) 拍照 → GroundingDINO
            #         识别 "white square." 桌面 → SAM2AutomaticMaskGenerator
            #         分割桌上各物品
            self.get_logger().info("Step 1: Detect desk objects")
            results = self.step_detect_desk_objects()
            if not results:
                self.get_logger().warn("No desk objects detected, retrying …")
                time.sleep(0.5)
                try_times -= 1
                continue
            try_times = 3

            # Step 2: GraspNet 检测抓取, 过滤掉夹取中心不落在 SAM
            #         mask 内的候选, 从 score 最高者开始抓取
            self.get_logger().info("Step 2: Detect grasps and grasp object")
            if not self.step_any_grasp(results):
                self.get_logger().warn("Grasp failed, re-detecting …")
                try_times -= 1
                continue
            time.sleep(0.5)

            robot_mode = self._robot_ctrl.get_robot_mode(self.pick_arm)
            if robot_mode == 6:  # Drag
                self._robot_ctrl.stop_drag(self.pick_arm)
            if robot_mode == 9:  # Error
                self._robot_ctrl.clear_error_arm(self.pick_arm)
                self._robot_ctrl.enable_arm(self.pick_arm)

            # Step 3: Go up
            self.get_logger().info("Go up")
            if not self.step_go_up():
                plan_result = self.mover.plan_named(self.pick_arm,"Home1")
                self.mover.execute(plan_result.trajectory)

            # Step 4: Place object
            self.get_logger().info("Place object")
            self.step_place_object()

        # node 已由 main() 的 MultiThreadedExecutor 常驻 spin,
        # 这里绝不能再 rclpy.spin_once(self): Jazzy 的 Executor.add_node
        # 无重复挂载保护, 会把 node 再挂到 global executor 与常驻
        # executor 并发 spin → callback 双重分发 (相机回调时序被搅乱)
        while rclpy.ok():
            time.sleep(0.1)

    # ------------------------------------------------------------------
    # Desk Detect: GroundingDINO + SAM2 → ROI → SAM2AutomaticMaskGenerator
    # ------------------------------------------------------------------
    def step_detect_desk_objects(self):
        """单帧桌面检测 + 物体点云高度中位数排序: 仿照 detect_desk_object.py。

        流程:
            1. 取 ``_DESK_CAMERA`` 最新彩色帧与有序彩色点云
            2. GroundingDINO 检测 ``_DESK_PROMPT`` (默认 "white square.")
               → SAM2 切分得到白色桌面的 mask
            3. 由 mask 外接矩形 (±``_DESK_ROI_SHRINK_PX``) 得到 ROI
            4. 在 ROI 裁剪区域内运行 ``SAM2AutomaticMaskGenerator``
               → 桌上物体 mask 列表 (按面积降序, #0 通常是桌面本身)
            5. 排除面积最大的 mask (``_DESK_DROP_LARGEST_MASK``), 并排除
               未完整落在步骤 2 桌面 SAM 外轮廓以内的 mask
            6. 把每个物体的 mask 映射到点云, 取 z 中位数作为高度
            7. 按 ``(height_median, index)`` 升序排序 (z 越小=越靠近相机=
               物理越高, None 排末尾), 在 OpenCV 窗口显示

        Returns
        -------
        list[dict] or None or False
            - ``False``: 用户按 'q' 请求退出循环
            - ``None``: 无彩色帧, 跳过本轮
            - ``list[dict]``: 排序后的物体列表, 每项含:
                - ``index``: int, 原始序号 (面积降序, 去桌面后的 0 基)
                - ``roi``: tuple (x0, y0, x1, y1), 全图坐标 bbox
                - ``mask``: np.ndarray (H, W) bool, 全图掩码
                - ``height_median``: float or None, 相机光学系 z 中位数 (m)
        """
        # ── 1. 取最新彩色帧 ──────────────────────────────────
        image = self.get_latest_color(_DESK_CAMERA)
        if image is None:
            self.get_logger().warn(f"[{_DESK_CAMERA}] no color frame yet",throttle_duration_sec=2.0)
            return None
        H, W = image.shape[:2]

        # ── 2. GroundingDINO + SAM2 检测桌面 ─────────────────
        det = self.dino.detect(image, _DESK_PROMPT)
        table_mask, roi = self._compute_table_roi(det, H, W)

        # ── 3. ROI 内 SAM2 自动掩码生成 ──────────────────────
        anns = []
        if roi is not None:
            anns = self._run_auto_masks(image, roi)

        # ── 4. 排除面积最大的 mask (桌面), 剩余为各个物体 ───
        if _DESK_DROP_LARGEST_MASK and len(anns) > 1:
            objects = anns[1:]
        else:
            objects = anns
        # ROI 是桌面 mask 的外接矩形, 其四角仍可能位于真实桌面轮廓外。仅保留完整落在步骤 2 桌面 SAM 最大外轮廓内部的候选物体。
        objects, excluded_count = self._filter_masks_inside_table(
            objects, table_mask
        )
        if excluded_count:
            self.get_logger().info(f"Excluded {excluded_count} object mask(s) outside the table SAM outer contour.")

        # ── 5. 取点云 → 计算每个物体 z 中位数 (高度) ─────────
        xyz = None
        valid = None
        H_pc = W_pc = 0
        with self._snapshots_lock:
            cloud_msg = self._snapshots.get(_DESK_CAMERA, {}).get("points")
        if cloud_msg is not None:
            try:
                xyz, _rgb, H_pc, W_pc = self._ros_pointcloud_to_organized(cloud_msg)
                valid = np.isfinite(xyz).all(axis=2) & (xyz[..., 2] > 0)
            except Exception as exc:
                self.get_logger().warn(f"[{_DESK_CAMERA}] point cloud parse failed: {exc}",throttle_duration_sec=2.0,)
                xyz = None

        # ── 6. 构建结果列表 (index, roi, mask, height_median) ─
        results = []
        for i, a in enumerate(objects):
            mask = a["segmentation_full"]
            height_median = None
            if xyz is not None and valid is not None:
                mask_pc = mask
                if mask_pc.shape != (H_pc, W_pc):
                    mask_pc = cv2.resize(mask.astype(np.uint8), (W_pc, H_pc),interpolation=cv2.INTER_NEAREST,).astype(bool)
                sel = mask_pc & valid
                if np.any(sel):
                    height_median = float(np.median(xyz[sel][:, 2]))
            results.append({
                "index": i,
                "roi": tuple(int(v) for v in a["bbox_xyxy_full"]),
                "mask": mask,
                "height_median": height_median,
            })

        # ── 7. 排序: height_median 升序 (z 越小=物理越高在前),
        #         None 排末尾; 同值按 index ───────────────────
        def _sort_key(r):
            h = r["height_median"]
            if h is None:
                return (1, 0.0, r["index"])
            return (0, h, r["index"])

        results.sort(key=_sort_key)

        # ── 8. 日志 ──────────────────────────────────────────
        if results:
            self.get_logger().info(
                f"—— {_DESK_CAMERA} desk objects: {len(results)} "
                f"(sorted by height) ——"
            )
            for rank, r in enumerate(results):
                h = r["height_median"]
                h_str = f"{h:.4f}" if h is not None else "N/A"
                x0, y0, x1, y1 = r["roi"]
                self.get_logger().info(
                    f"rank={rank} idx={r['index']} "
                    f"roi=({x0},{y0})-({x1},{y1}) height_z={h_str}"
                )

        # ── 9. OpenCV 可视化 ────────────────────────────────
        self.get_logger().info("Step_1a")
        display = self._draw_desk_objects(image, table_mask, roi, results)
        cv2.putText(
            display,
            f"dets={len(det.boxes)} roi={'yes' if roi is not None else 'no'} "
            f"objects={len(results)}",
            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2,
        )
        cv2.imshow(_DESK_WINDOW_NAME, display)
        self.get_logger().info("Step_1b")
        key = cv2.waitKey(1) & 0xFF
        self.get_logger().info("Step_1c")
        if key == ord('q'):
            self.get_logger().info(
                "Pressed 'q' in OpenCV window, exiting desk detect loop …"
            )
            return False
        return results

    def _compute_table_roi(self, det, H: int, W: int):
        """由 GroundingDINO 检测结果的 mask 计算白色桌面 ROI。

        Returns
        -------
        tuple[np.ndarray, tuple] or tuple[None, None]
            ``(mask2d, (x0, y0, x1, y1))``: ``mask2d`` 为 ``(H, W)`` bool 桌面掩码,
            roi 为按 ``_DESK_ROI_SHRINK_PX`` 内缩后的外接矩形 (全图坐标, 已 clamp)。
        """
        if len(det.boxes) == 0 or len(det.masks) == 0:
            return None, None

        mask2d = np.any(det.masks, axis=0)
        if not np.any(mask2d):
            return None, None

        ys, xs = np.where(mask2d)
        x0, y0, x1, y1 = xs.min(), ys.min(), xs.max(), ys.max()

        s = int(_DESK_ROI_SHRINK_PX)
        x0, y0 = x0 + s, y0 + s
        x1, y1 = x1 - s, y1 - s

        x0 = max(0, min(W - 1, x0))
        y0 = max(0, min(H - 1, y0))
        x1 = max(x0 + 1, min(W - 1, x1))
        y1 = max(y0 + 1, min(H - 1, y1))
        return mask2d, (int(x0), int(y0), int(x1), int(y1))

    @staticmethod
    def _filter_masks_inside_table(objects, table_mask):
        """仅保留完整位于桌面 SAM 最大外轮廓内部的候选 mask。"""
        if table_mask is None or not np.any(table_mask):
            return [], len(objects)

        contours, _ = cv2.findContours(
            table_mask.astype(np.uint8),
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )
        if not contours:
            return [], len(objects)

        outer_contour = max(contours, key=cv2.contourArea)
        table_inside = np.zeros(table_mask.shape, dtype=np.uint8)
        cv2.drawContours(
            table_inside, [outer_contour], -1, 1, thickness=cv2.FILLED
        )
        table_inside = table_inside.astype(bool)

        kept = []
        for obj in objects:
            mask = np.asarray(obj["segmentation_full"], dtype=bool)
            if mask.shape != table_inside.shape or not np.any(mask):
                continue
            if np.any(mask & ~table_inside):
                continue
            kept.append(obj)

        return kept, len(objects) - len(kept)

    def _run_auto_masks(self, frame: np.ndarray, roi: tuple):
        """在 ROI 裁剪区域内运行 SAM2AutomaticMaskGenerator。

        Parameters
        ----------
        frame : np.ndarray
            BGR 全图
        roi : tuple
            ``(x0, y0, x1, y1)`` 全图坐标

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
        min_area = int(_DESK_MIN_AREA_RATIO * roi_area)

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

        out.sort(key=lambda a: a["area"], reverse=True)
        return out

    def _draw_desk_objects(
        self, display: np.ndarray, table_mask, roi, results
    ) -> np.ndarray:
        """OpenCV 可视化: 桌面 mask (绿) + ROI (黄) + 每个物体轮廓/bbox/编号/高度。

        Parameters
        ----------
        results : list[dict]
            已排序的物体列表, 每项含 index/roi/mask/height_median。
            显示标签用排序后的 rank (与返回顺序一致)。
        """
        if table_mask is not None:
            overlay = display.copy()
            overlay[table_mask] = (0, 255, 0)
            display = cv2.addWeighted(display, 0.7, overlay, 0.3, 0)

        if roi is not None:
            x0, y0, x1, y1 = roi
            cv2.rectangle(display, (x0, y0), (x1, y1), (0, 255, 255), 2)

        for rank, r in enumerate(results):
            mask = r["mask"]
            mask_u8 = mask.astype(np.uint8)
            cnts, _ = cv2.findContours(
                mask_u8, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )
            cv2.drawContours(display, cnts, -1, (255, 0, 0), 2)

            x0, y0, x1, y1 = r["roi"]
            cv2.rectangle(display, (x0, y0), (x1, y1), (255, 128, 0), 1)
            cx = int((x0 + x1) / 2)
            cy = int((y0 + y1) / 2)
            h = r["height_median"]
            h_str = f"{h:.3f}" if h is not None else "NA"
            cv2.putText(
                display, f"#{rank} h={h_str}", (cx, cy),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2,
            )

        return display


        


    # ------------------------------------------------------------------
    # Step 2 Any Grasp: 点云解析 + GraspNet 相关
    # ------------------------------------------------------------------
    @staticmethod
    def _tip_toward_tcp_offset(pose: Pose, offset_m: float) -> Pose:
        """将 tip 规划点沿 pose 局部 -Z (tip→tcp 方向) 平移 offset_m 米。

        URDF 中 tip 位于 L*_6 局部 Z 轴 0.195 m, tcp 位于 0.14568 m,
        即 tcp 在 tip 后方 0.04932 m 处。offset_m 取 [0, 0.04932] 时,
        结果等价于"把规划目标放在 tip 与 tcp 之间"——配合 tip_link 规划后,
        真实抓取中心将落在偏移前的目标点上 (tip 偏高低、tcp 偏高, 取中间值即可)。

        Parameters
        ----------
        pose : Pose
            原 tip 规划位姿 (base_link 下).
        offset_m : float
            tip 往 tcp 方向偏移的米数 (正值 → 向 tcp 后退).

        Returns
        -------
        Pose
            平移后的新位姿, 姿态保持不变.
        """
        z_axis = R.from_quat([
            pose.orientation.x, pose.orientation.y,
            pose.orientation.z, pose.orientation.w,
        ]).apply([0.0, 0.0, 1.0])
        out = Pose()
        out.position.x = pose.position.x - z_axis[0] * offset_m
        out.position.y = pose.position.y - z_axis[1] * offset_m
        out.position.z = pose.position.z - z_axis[2] * offset_m
        out.orientation = pose.orientation
        return out

    # J6 角度限位 (URDF ±6.28 rad ≈ ±360°), 有界回转关节, 不可回环
    _J6_LIMIT_RAD = 6.28

    def _flip_j6_if_needed(self, plan_result, current_joints, arm, candidate_idx):
        """选择使 J6 真实行程最小的等价目标, 必要时关节空间重规划。

        J6 是有界回转关节 [-360°, 360°], 不可回环: 两个"朝向很近"的关节
        值在关节空间里可能差 300°~380°。同一末端朝向有多个代表值 (相差
        ±360° 缠绕), 叠加平行夹爪的 180° 抓取对称性, 等价 J6 目标每 180°
        一个。原始 IK 解不一定是离 current 最近者, 直接执行可能产生大旋转。

        做法: 枚举所有合法等价目标 (缠绕 ±360° ∪ 抓取对称 ±180°, 限内),
        按「原始关节差」(规划器实际执行量, 非归一化) 取离 current 最近者;
        若与原 IK 解不同则重规划。因候选每 180° 一个, 最近者真实行程必
        ≤90° (关节限位边界除外, 此时取可达最近者)。
        """
        try:
            traj_msg = plan_result.trajectory.get_robot_trajectory_msg()
            jt = traj_msg.joint_trajectory
            if not jt.points:
                return plan_result
            final_by_name = dict(zip(jt.joint_names, jt.points[-1].positions))
        except Exception:
            return plan_result

        arm_prefix = 'J1' if arm == 'Arm1' else 'J2'
        active_names = [f'{arm_prefix}_{i}' for i in range(1, 7)]
        if not all(n in final_by_name for n in active_names):
            return plan_result

        final_joints = [final_by_name[n] for n in active_names]
        ik_j6 = final_joints[5]
        cur_j6 = current_joints[5]
        raw_motion = ik_j6 - cur_j6  # 原方案规划器实际执行的关节行程

        # 枚举等价 J6 目标: {ik, ik+π} 各加 ±360° 缠绕, 仅保留限内
        candidates = set()
        for base in (ik_j6, ik_j6 + math.pi):
            for k in (-1, 0, 1):
                c = base + k * 2 * math.pi
                if -self._J6_LIMIT_RAD <= c <= self._J6_LIMIT_RAD:
                    candidates.add(round(c, 6))
        if not candidates:
            return plan_result

        # 按离 current 的原始行程升序; 最近者即最优等价目标
        ordered = sorted(candidates, key=lambda c: abs(c - cur_j6))
        best = ordered[0]
        if abs(best - ik_j6) < 1e-6:
            # 原 IK 解已是最优 → 真实行程本就 ≤90°, 无需重规划
            return plan_result

        best_motion = best - cur_j6
        self.get_logger().info(
            f"Candidate {candidate_idx + 1}: J6 原始行程 "
            f"{math.degrees(raw_motion):.1f}° 过大, 选等价目标 → "
            f"真实行程 {math.degrees(best_motion):.1f}°, 关节空间重规划"
        )

        # 依次尝试候选 (最近优先), 任一成功即用; 全失败则回退原方案
        for c in ordered:
            if abs(c - ik_j6) < 1e-6:
                continue
            target = list(final_joints)
            target[5] = c
            replan = self.mover.plan_joints(
                arm, target, planner=Planner.pilz_ptp,
            )
            if replan is not None:
                return replan

        self.get_logger().warn(
            f"Candidate {candidate_idx + 1}: J6 等价目标重规划全部失败, "
            "使用原方案"
        )
        return plan_result

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

    def _grasp_group_to_base_poses(
        self, gg, frame_id: str, stamp,
        object_mask=None, camera_info=None,
    ):
        """过滤/排序 GraspGroup → 取前 10 → 轴重映射 → TF 变换到 base_link。

        若提供 ``object_mask`` (图像分辨率 bool, SAM 物体并集) 与
        ``camera_info``, 会先过滤掉夹取中心 (反投影像素) 不落在 mask
        内的候选, 再做 NMS / 角度过滤 / top-K。

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
            self.get_logger().error("No grasp pose due to max depth > _GRASP_MAX_DEPTH")
            return [], [], []

        # ── 过滤: 夹取中心反投影像素必须落在 SAM 物体 mask 并集内 ──
        # grasp.translation 位于点云坐标系 (= 相机 color optical frame),
        # 用内参反投影到像素后查 object_mask。
        if object_mask is not None and camera_info is not None and len(gg) > 0:
            fx, fy = camera_info.k[0], camera_info.k[4]
            cx, cy = camera_info.k[2], camera_info.k[5]
            mh, mw = object_mask.shape[:2]
            kept = []
            for i, g in enumerate(gg):
                gx, gy, gz = (
                    float(g.translation[0]),
                    float(g.translation[1]),
                    float(g.translation[2]),
                )
                if gz <= 1e-6:
                    continue
                u = int(round(gx / gz * fx + cx))
                v = int(round(gy / gz * fy + cy))
                if 0 <= u < mw and 0 <= v < mh and object_mask[v, u]:
                    kept.append(i)
            if len(kept) == 0:
                self.get_logger().error(
                    "No grasp pose: all grasp centers outside SAM object masks."
                )
                return [], [], []
            self.get_logger().info(
                f"Mask-center filter: kept {len(kept)}/{len(gg)} grasps."
            )
            gg = gg[kept]

        gg.nms()
        gg.sort_by_score()

        if _GRASP_ANGLE_LIMIT > 0:
            try:
                cam_to_base = self._tf_buffer.lookup_transform(
                    "base_link", frame_id, stamp,
                    timeout=rclpy.duration.Duration(seconds=1.0),
                )
                rot = cam_to_base.transform.rotation
                R_c2b = R.from_quat(
                    [rot.x, rot.y, rot.z, rot.w]
                ).as_matrix()
                world_z_in_cam = R_c2b.T @ np.array(
                    [0.0, 0.0, 1.0]
                )
            except Exception:
                self.get_logger().warn(
                    f"Angle filter skipped: "
                    f"cannot get {frame_id}→base_link TF",
                    throttle_duration_sec=5.0,
                )
            else:
                kept = []
                for i, g in enumerate(gg):
                    tool_R = g.rotation_matrix @ _GRASP_TO_TOOL
                    approach = tool_R[:, 2]
                    cos_ang = np.dot(approach, world_z_in_cam)
                    ang_deg = np.degrees(
                        np.arccos(
                            np.clip(np.abs(cos_ang), 0.0, 1.0)
                        )
                    )
                    if ang_deg <= _GRASP_ANGLE_LIMIT:
                        kept.append(i)
                if len(kept) == 0:
                    self.get_logger().error("No grasp pose due to ang_deg < _GRASP_ANGLE_LIMIT")
                    return [], [], []
                gg = gg[kept]

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
                    # 仅在几何更新后重绘。原实现每 10ms 无条件
                    # update_renderer (~100Hz 全量渲染), 即使画面无变化,
                    # 常驻抢占 GIL/CPU, 挤压相机等高频回调的执行窗口。
                    vis.update_renderer()

                if not vis.poll_events():
                    break
                time.sleep(0.01 if pending is not None else 0.05)

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
        """在常驻 Open3D 窗口中显示执行成功的夹爪 (复用单窗口)。

        原实现每次抓取成功都新起线程 + 新建 Open3D/GLFW 窗口:
        连续抓取 2~3 次后, 进程内同时存在多个 GL 上下文/多个可视化
        线程, 与常驻窗口线程并发调用 GLFW (非线程安全) → 段错误
        (exit code -11, 且崩溃点总在 "Step 2 completed" 日志之后)。

        现改为把「点云 + 成功的那个夹爪」推送到 _show_grasps_o3d 的
        常驻窗口 (替代之前的 top-10 显示), 全进程只有一个可视化线程、
        一个 GL 上下文, 不再崩溃。点云与夹爪均在相机 optical frame,
        坐标系一致可直接叠加。按 Esc/Q 或关闭窗口退出。
        """
        geos = [] if gripper_geo is None else [gripper_geo]
        self.get_logger().info(
            f"Success grasp (score={score:.3f}) shown in persistent window."
        )
        self._show_grasps_o3d(cloud_o3d, geos)

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

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        node.launch()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()