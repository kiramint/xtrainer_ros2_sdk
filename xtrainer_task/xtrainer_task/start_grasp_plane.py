"""Entry point for xtrainer_task — robot movement using MoveIt.

Configs (URDF, SRDF, ompl_planning, etc.) are expected to be loaded
from ROS 2 parameters, which are set by the launch file:
    ros2 launch xtrainer_task start.launch.py

Usage
-----
    ros2 launch xtrainer_task start.launch.py
"""


import math
import sys
import threading
import time
import traceback
from collections import deque
from typing import Dict, Optional, Tuple

import cv2
import numpy as np
import rclpy
import tf2_geometry_msgs
import tf2_ros
from geometry_msgs.msg import PointStamped, Pose, PoseStamped
from moveit.planning import MoveItPy
from rclpy.callback_groups import (MutuallyExclusiveCallbackGroup,
                                   ReentrantCallbackGroup)
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sam2.automatic_mask_generator import SAM2AutomaticMaskGenerator
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from visualization_msgs.msg import Marker

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
_ARM_TCP = {
    "Arm1": "L1_gripper_tcp",
    "Arm2": "L2_gripper_tcp",
}
_ARM_HOME = {
    "Arm1": "Home1",
    "Arm2": "Home2",
}

_ARM_CAM = {
    "Arm1": "camera_left",
    "Arm2": "camera_right",
}
_GRIPPER = {
    "Arm1": "left",
    "Arm2": "right",
}

# ── Step 2 Any Grasp: GroundingDINO + SAM2 + minAreaRect 几何抓取 ───────
_GRASP_PC_TOPIC_TEMPLATE = "/camera/{name}/depth/color/points"
_GRASP_PROMPT = "object."
# URDF: tip=L*_6+Z0.195, tcp=L*_6+Z0.14568 → 向下安装时 tip 比 tcp 低 0.04932 m
_TCP_TIP_OFFSET_M = 0.04932
# minAreaRect 长轴 XY 投影最短长度 (m), 过短则方向不可靠, 重试
_ANY_AXIS_MIN_LEN_M = 0.01

_GRASP_TIP_Z_LIMIT = 0.0461
_GRASP_TIP_TO_TCP = 0.04932 # Distance between tip and tcp

# ── Desk Detect: GroundingDINO + SAM2 白色桌面 → ROI → 自动掩码物体分割 ─
_DESK_CAMERA = "camera_top"
_DESK_PROMPT = "white square."
_DESK_ROI_SHRINK_PX = 0           # ROI 整体向内缩 x 像素 (正值缩小)
_DESK_MIN_AREA_RATIO = 0.002      # 自动掩码面积下限 (相对 ROI 面积比例)
_DESK_DROP_LARGEST_MASK = True    # 排除面积最大的 mask (桌子表面本身)
_DESK_WINDOW_NAME = "Desk Object Detect"

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


def fit_rectangle_orientation(mask: np.ndarray):
    """对二值 mask 做最小外接矩形拟合, 返回物体较长边的方向向量。

    使用 ``cv2.minAreaRect`` 对 mask 的所有前景像素拟合最小面积外接矩形,
    再由矩形 4 个角点计算两条邻边向量, 取较长者作为物体主轴方向。

    Parameters
    ----------
    mask : np.ndarray
        ``(H, W)`` bool 数组, True 为物体像素。

    Returns
    -------
    dict or None
        拟合失败 (前景像素 < 3) 返回 None。成功返回::

            {
                "rect":   ((cx, cy), (w, h), angle),   # cv2.minAreaRect 原始返回
                "box":    np.ndarray (4, 2) float32,   # cv2.boxPoints 4 角点
                "center": np.ndarray (2,) float32,     # 矩形中心 (cx, cy)
                "axis":   np.ndarray (2,) float32,     # 较长边单位方向向量 (dx, dy)
                "length": float,                       # 较长边长度 (像素)
                "angle_deg": float,                    # 方向向量与 +X 轴夹角 (度)
            }
    """
    ys, xs = np.where(mask)
    if len(xs) < 3:
        return None

    # (N, 2) 像素坐标 (x, y)
    points = np.stack([xs, ys], axis=1).astype(np.float32)

    # 最小外接矩形: ((cx, cy), (w, h), angle), angle ∈ [-90, 0)
    rect = cv2.minAreaRect(points)
    box = cv2.boxPoints(rect)  # (4, 2) 顺时针 4 角点

    center = np.array(rect[0], dtype=np.float32)

    # 两条邻边向量
    e1 = box[1] - box[0]
    e2 = box[2] - box[1]
    l1 = float(np.linalg.norm(e1))
    l2 = float(np.linalg.norm(e2))

    if l1 >= l2:
        long_vec = e1
        long_len = l1
    else:
        long_vec = e2
        long_len = l2

    axis = long_vec / long_len if long_len > 0 else long_vec
    angle_deg = math.degrees(math.atan2(axis[1], axis[0]))

    return {
        "rect": rect,
        "box": box,
        "center": center,
        "axis": axis,
        "length": long_len,
        "angle_deg": angle_deg,
    }


def _circ_angle_diff(a: float, b: float) -> float:
    """两个角度的圆周差, 归一化到 [-π, π]。"""
    return (a - b + math.pi) % (2 * math.pi) - math.pi


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

        # Give MoveIt some time to receive latest /joint_states and TF
        self.get_logger().info('Waiting for MoveIt state to populate …')
        time.sleep(2.0)

        # DINO 检测器 (SAM2 用 base_plus, 比 large 省显存/更快)
        self.dino = DinoWrapper(
            device='cuda', box_threshold=0.35, text_threshold=0.25,
            sam2_checkpoint="/opt/Project/Grounded-SAM-2/checkpoints/sam2.1_hiera_base_plus.pt",
            sam2_config="configs/sam2.1/sam2.1_hiera_b+.yaml",
        )

        # SAM2 自动掩码生成器 (复用 DinoWrapper 加载的 sam2_model 权重)
        # 用于在白色桌面 ROI 内自动切分桌上各个物体
        # 参数与 sam_detection_test.py 默认一致 (base_plus + 32 点 + 单层裁剪)
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

        # ── 各相机最新帧快照 (单锁, 每帧覆盖) ──
        self._snapshots: Dict[str, dict] = {}
        self._snapshots_lock = threading.Lock()

        # ── 相机内参缓存 (camera_name → CameraInfo) ──
        self._camera_infos: Dict[str, CameraInfo] = {}
        self._camera_info_lock = threading.Lock()

        # ── 桌面物体检测最新结果 (launch 循环每帧刷新, 供外部读取) ──
        self._latest_desk_objects = None

        # ARM
        self.pick_arm = "Arm1"

        # ── TF2 缓冲与监听器 ──
        # cache_time 调大到 120s: Step2 的 SAM2 推理 ~1s, 但若 TF 失败进入
        # while 重试, 累积可达数十秒. 默认 10s cache 会在 SAM 完成前就把点云
        # 采集时刻的 TF 丢弃, 导致 "Lookup would require extrapolation into
        # the past".
        self._tf_buffer = tf2_ros.Buffer(
            cache_time=rclpy.duration.Duration(seconds=120)
        )
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._camera_cb_group = ReentrantCallbackGroup()
        self._cloud_cb_group = MutuallyExclusiveCallbackGroup()


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

    def step_approach_object(self,camera_name, mid_x, mid_y):
        mid_coordinate = self.pixel_to_base_link(camera_name, mid_x, mid_y)             
        if mid_coordinate is None:
            self.get_logger().error("Failed to compute 3D coordinate of object center.")
            return False
            
        self._publish_detection_marker(mid_coordinate)

        # Pose gripper down and 5cm above the object center
        pose_prepare = Pose()
        pose_prepare.position.x = mid_coordinate[0]
        pose_prepare.position.y = mid_coordinate[1]
        pose_prepare.position.z = mid_coordinate[2] + 0.1
        pose_prepare.orientation.x = 0.9999995231628418
        pose_prepare.orientation.y = 7.932441803859547e-06
        pose_prepare.orientation.z = 0.001016218215227127
        pose_prepare.orientation.w = 9.80220761448436e-07

        self.get_logger().info(f"############# Move to pose {pose_prepare} ###############")

        # Try every arm
        for _ in range(2):
            # move arm
            plan_result = self.mover.plan_pose(self.pick_arm, pose_prepare, planner=Planner.ompl,tip_link=_ARM_TIP[self.pick_arm])

            if plan_result is None:
                self.get_logger().error(f"################### Moveit {self.pick_arm} Planning Failed #################")
                if self.pick_arm == "Arm1":
                    self.pick_arm = "Arm2"
                else:
                    self.pick_arm = "Arm1"
            else:
                break

        if plan_result is not None:
            self.mover.execute(plan_result.trajectory)
            time.sleep(0.5)
            return True
        else:
            return False

    def step_any_grasp(self):
        """顶抓任意物体: GroundingDINO+SAM2 → minAreaRect → TF 真实方向 →
        生成抓取姿态 → TCP 降到物体高度 → 闭合夹爪。

        前置: 机械臂已由 step_approach_object 移到物体上方, 手上相机俯视物体。

        流程:
            1. 手上相机检测 ``_GRASP_PROMPT`` + SAM2 切分, 选最靠近画面中心者
            2. 对 mask 做 ``cv2.minAreaRect`` 拟合; 取长轴两端点各自经
               ``pixel_to_base_link`` (深度+TF) 变换到 base_link 求真实 3D 方向
               —— 每个 endpoint 独立取深度+TF 即对斜拍相机的梯形矫正
            3. yaw = 长轴角 ±90° 归一化; 夹爪 x 轴 ⊥ 物体长轴 (侧夹);
               roll=-π 顶抓, pitch=0
            4. 物体中心高度 h_obj 由中心像素深度+TF 得到; tip 目标 =
               h_obj - _TCP_TIP_OFFSET_M (使 tcp 落到物体高度), 并夹紧
               tip.z ≥ _GRASP_Z_LIMIT; 先在当前高度旋转手腕+对准 xy
               (4a, ``_flip_j6_if_needed``), 再垂直下移到抓取高度 (4b)
            5. 闭合夹爪
        """
        hand_cam = _ARM_CAM[self.pick_arm]
        try_time = 3

        while rclpy.ok() and try_time:
            try_time -= 1
            # ── 1. GroundingDINO + SAM2 检测, 选画面中心 mask ──
            image = self.get_latest_color(hand_cam)
            if image is None:
                self.get_logger().error(
                    f"No {hand_cam} camera image available.")
                time.sleep(0.05)
                continue

            result = self.dino.detect(image, _GRASP_PROMPT)
            if len(result.boxes) == 0:
                self.get_logger().warn(
                    f"No '{_GRASP_PROMPT}' detected for grasp.")
                time.sleep(0.05)
                continue

            try:
                annotated_image = self.dino.annotate(
                    image, result, draw_mask=True)
            except Exception:
                annotated_image = image
            cv2.imshow("Detection Result", annotated_image)
            cv2.waitKey(1)

            H_img, W_img = image.shape[:2]
            mask_img = self._pick_center_mask(result, H_img, W_img)
            if mask_img is None:
                self.get_logger().warn("No usable center mask.")
                time.sleep(0.05)
                continue

            # ── 2. minAreaRect 长轴 → TF 真实 3D 方向 (梯形矫正) ──
            fit = fit_rectangle_orientation(mask_img)
            if fit is None:
                self.get_logger().warn(
                    "minAreaRect fit failed (mask too small).")
                time.sleep(0.05)
                continue
            axis = fit["axis"]
            half_len = fit["length"] / 2.0
            cx_img = float(fit["center"][0])
            cy_img = float(fit["center"][1])
            p0_img = (cx_img - axis[0] * half_len, cy_img - axis[1] * half_len)
            p1_img = (cx_img + axis[0] * half_len, cy_img + axis[1] * half_len)

            p0 = self.pixel_to_base_link(
                hand_cam, p0_img[0], p0_img[1], window=3)
            p1 = self.pixel_to_base_link(
                hand_cam, p1_img[0], p1_img[1], window=3)
            c_base = self.pixel_to_base_link(
                hand_cam, cx_img, cy_img, window=5)
            if p0 is None or p1 is None or c_base is None:
                self.get_logger().warn(
                    "TF/depth lookup failed for object axis/center; retry.")
                time.sleep(0.05)
                continue

            direction = np.asarray(p1, dtype=float) - np.asarray(p0, dtype=float)
            dir_xy = direction[:2]
            len_xy = float(np.linalg.norm(dir_xy))
            if len_xy < _ANY_AXIS_MIN_LEN_M:
                self.get_logger().warn(
                    f"Object 3D axis too short ({len_xy:.4f} m); retry.")
                time.sleep(0.05)
                continue
            dir_xy = dir_xy / len_xy

            h_obj = float(c_base[2])

            # ── 3. 抓取姿态: 顶抓, 夹爪 x ⊥ 物体长轴 ──
            object_yaw = math.atan2(dir_xy[1], dir_xy[0])
            # 归一化到 [-π/2, π/2] (对称夹爪 θ ≡ θ ± π)
            if object_yaw > math.pi / 2:
                object_yaw -= math.pi
            elif object_yaw <= -math.pi / 2:
                object_yaw += math.pi
            grasp_yaw = object_yaw - math.pi / 2  # 垂直长轴

            # 在 grasp_pose 阶段就选定 180° 分支: 以当前末端朝向为参考,
            # 在 grasp_yaw 与 grasp_yaw+π 中取离当前 tool yaw 最近者。
            # 4a/4b 用同一 orientation, 避免 plan 级 flip 偷偷转到 R' 分支
            # 导致 4b 反向旋转 180°。
            cur_pose = self.mover.get_current_pose(
                self.pick_arm, tip_link=_ARM_TCP[self.pick_arm])
            cur_R = R.from_quat([
                cur_pose.orientation.x, cur_pose.orientation.y,
                cur_pose.orientation.z, cur_pose.orientation.w,
            ]).as_matrix()
            # 末端 x 轴在世界 XY 平面的偏角 = 当前 tool yaw
            cur_yaw = math.atan2(cur_R[1, 0], cur_R[0, 0])
            if abs(_circ_angle_diff(grasp_yaw, cur_yaw)) <= abs(
                _circ_angle_diff(grasp_yaw + math.pi, cur_yaw)
            ):
                chosen_yaw = grasp_yaw
            else:
                chosen_yaw = grasp_yaw + math.pi

            quat = R.from_euler(
                "xyz", [-math.pi, 0.0, chosen_yaw]
            ).as_quat()

            self.get_logger().info(
                f"Object center=({c_base[0]:.3f},{c_base[1]:.3f},"
                f"{c_base[2]:.3f}) axis_xy=({dir_xy[0]:.3f},{dir_xy[1]:.3f}) "
                f"object_yaw={math.degrees(object_yaw):.1f}° "
                f"grasp_yaw={math.degrees(grasp_yaw):.1f}° "
                f"chosen_yaw={math.degrees(chosen_yaw):.1f}° "
                f"cur_yaw={math.degrees(cur_yaw):.1f}° h_obj={h_obj:.4f}"
            )

            grasp_pose = Pose()
            grasp_pose.position.x = c_base[0]
            grasp_pose.position.y = c_base[1]
            grasp_pose.position.z = h_obj
            grasp_pose.orientation.x = float(quat[0])
            grasp_pose.orientation.y = float(quat[1])
            grasp_pose.orientation.z = float(quat[2])
            grasp_pose.orientation.w = float(quat[3])

            # ── 4. 先旋转末端对准姿态, 再垂直下移 (分两步) ──
            # tip 比 tcp 低 _TCP_TIP_OFFSET_M; 欲 tcp 落到 h_obj → tip = h_obj - offset
            if grasp_pose.position.z < _GRASP_TIP_Z_LIMIT + _GRASP_TIP_TO_TCP:
                grasp_pose.position.z = _GRASP_TIP_Z_LIMIT + _GRASP_TIP_TO_TCP

            # 4a. 在当前高度旋转手腕 + 对准物体 xy (不下降)
            # cur_pose 已在 step 3 读取 (两步之间无运动, 仍有效)
            pre_pose = Pose()
            pre_pose.position.x = grasp_pose.position.x
            pre_pose.position.y = grasp_pose.position.y
            pre_pose.position.z = cur_pose.position.z  # 保持当前高度
            pre_pose.orientation = grasp_pose.orientation
            self._publish_detection_marker(
                (pre_pose.position.x, pre_pose.position.y, pre_pose.position.z))

            current_joints = self.mover.get_current_joint_positions(
                self.pick_arm)
            plan_rot = self.mover.plan_pose(
                self.pick_arm, pre_pose, planner=Planner.pilz_lin,
                tip_link=_ARM_TCP[self.pick_arm])
            if plan_rot is None:
                plan_rot = self.mover.plan_pose(
                    self.pick_arm, pre_pose, planner=Planner.pilz_ptp,
                    tip_link=_ARM_TCP[self.pick_arm])
            if plan_rot is None:
                self.get_logger().warn("Rotate plan failed; re-detecting.")
                time.sleep(0.05)
                continue

            plan_rot = self._flip_j6_if_needed(
                plan_rot, current_joints, self.pick_arm, 0)
            if not self.mover.execute(plan_rot.trajectory):
                self.get_logger().warn("Rotate execute failed; re-detecting.")
                time.sleep(0.05)
                continue
            self.get_logger().info("Rotated to grasp orientation; descending.")

            # 4b. 垂直下移到抓取高度 (grasp_pose: xy/姿态不变, 仅 z 下降)
            # 4a 已移动, 重取当前关节作为 4b flip 的 ±360° 缠绕兜底基准
            current_joints = self.mover.get_current_joint_positions(
                self.pick_arm)
            self._publish_detection_marker(
                (grasp_pose.position.x, grasp_pose.position.y, grasp_pose.position.z))
            plan_desc = self.mover.plan_pose(
                self.pick_arm, grasp_pose, planner=Planner.pilz_lin,
                tip_link=_ARM_TCP[self.pick_arm])
            if plan_desc is None:
                plan_desc = self.mover.plan_pose(
                    self.pick_arm, grasp_pose, planner=Planner.pilz_ptp,
                    tip_link=_ARM_TCP[self.pick_arm])
            if plan_desc is None:
                self.get_logger().warn("Descend plan failed; re-detecting.")
                time.sleep(0.05)
                continue
            plan_desc = self._flip_j6_if_needed(
                plan_desc, current_joints, self.pick_arm, 1)
            if not self.mover.execute(plan_desc.trajectory):
                self.get_logger().warn("Descend execute failed; re-detecting.")
                time.sleep(0.05)
                continue

            try_time = 3
            self.get_logger().info(
                f"Step 2 completed: grasp pose reached with {self.pick_arm}.")
            break

        if try_time <= 0:
            time.sleep(0.05)
            plan_home = self.mover.plan_named(self.pick_arm,_ARM_HOME[self.pick_arm])
            self.mover.execute(plan_home.trajectory)
            time.sleep(0.05)
            if self.pick_arm == "Arm1":
                self.pick_arm = "Arm2"
            else:
                self.pick_arm = "Arm1"

        else:
            # ── 5. 闭合夹爪 ──
            self.gripper.close(_GRIPPER[self.pick_arm])

    def step_go_up(self):
        pose = self.mover.get_current_pose(self.pick_arm, tip_link=_ARM_TIP[self.pick_arm])
        z_limit = pose.position.z
        pose.position.z = z_limit + 0.2
        while pose.position.z > z_limit:
            plan_result = self.mover.plan_pose(self.pick_arm, pose, planner=Planner.pilz_lin, tip_link=_ARM_TIP[self.pick_arm])
            if plan_result is None:
                self.get_logger().error(f"################### Moveit {self.pick_arm} Planning Failed #################")
                pose.position.z -= 0.01
                time.sleep(0.1)
                continue
            else:
                self.mover.execute(plan_result.trajectory)
                self.get_logger().error(f"################### Moveit {self.pick_arm} Go Up #################")
                return True
        return False

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

            # Step 1: Detect desk objects
            self.get_logger().info("Step 1: Detect desk objects")
            results = self.step_detect_desk_objects()
            if results is None or len(results)==0:
                self.get_logger().warn("No desk objects detected, retrying …")
                time.sleep(0.5)
                try_times -= 1
                continue
            try_times = 3

            result = results[0]  # 取高度最低的物体
            roi = result["roi"]
            first_mid = ((roi[0] + roi[2]) // 2, (roi[1] + roi[3]) // 2)
            self.get_logger().info(f"First object ROI: {roi}, mid: {first_mid}")

            # Step 2: Approach object
            self.get_logger().info("Step 2: Approach object")
            self.step_approach_object("camera_top",first_mid[0], first_mid[1])

            # Step 3: Detect object and Grasp
            self.get_logger().info("Step 3: Detect object and Grasp")
            self.step_any_grasp()
            time.sleep(0.5)

            # Step 4: Go up
            self.get_logger().info("Go up")
            self.step_go_up()
            time.sleep(0.5)

            # Step 5: Place object
            self.get_logger().info("Step 5: Place object")
            while not self.step_place_object():
                time.sleep(0.01)

        time.sleep(0.01)

        plan_result_2 = self.mover.plan_named("Arm2","Home2")
        if plan_result_2 is not None:
            self.mover.execute(plan_result_2.trajectory)

        time.sleep(0.01)

        plan_result_1 = self.mover.plan_named("Arm1","Home1")
        if plan_result_1 is not None:
            self.mover.execute(plan_result_1.trajectory)

        rclpy.shutdown()

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
    # Step 2 Any Grasp: 几何抓取辅助 (tip/tcp 偏移, J6 翻转)
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

    def _pick_center_mask(
        self, result, H_pc: int, W_pc: int,
    ):
        """从检测结果中选出 mask 质心距离画面中心最近的单个 mask。

        对每个 detection mask 计算其质心 (图像坐标), 按与图像中心
        (W_img/2, H_img/2) 的欧氏距离升序排序, 返回最近的匹配 mask,
        resize 到点云分辨率。无候选返回 None。

        Parameters
        ----------
        result : DetectionResult
            GroundingDINO + SAM2 检测结果, ``result.masks`` 为 (N, H, W) bool。
        H_pc, W_pc : int
            点云高/宽。

        Returns
        -------
        np.ndarray or None
            (H_pc, W_pc) bool 单 mask (最靠近中心者); 无候选返回 None。
        """
        if len(result.boxes) == 0 or len(result.masks) == 0:
            return None

        masks = result.masks  # (N, H_img, W_img)
        H_img, W_img = masks.shape[1], masks.shape[2]
        cx_img, cy_img = W_img / 2.0, H_img / 2.0

        candidates = []
        for i in range(masks.shape[0]):
            m = masks[i]
            ys, xs = np.where(m)
            if len(xs) == 0:
                candidates.append((i, float("inf")))
                continue
            mx = float(np.mean(xs))
            my = float(np.mean(ys))
            dist = np.sqrt((mx - cx_img) ** 2 + (my - cy_img) ** 2)
            candidates.append((i, dist))

        if not candidates:
            return None

        candidates.sort(key=lambda c: c[1])

        chosen = candidates[0]
        m_chosen = masks[chosen[0]]
        if m_chosen.shape != (H_pc, W_pc):
            m_pc = cv2.resize(
                m_chosen.astype(np.uint8), (W_pc, H_pc),
                interpolation=cv2.INTER_NEAREST,
            ).astype(bool)
        else:
            m_pc = m_chosen

        self.get_logger().info(
            f"Pick center mask: idx={chosen[0]} "
            f"center_dist={chosen[1]:.1f}px (among {len(candidates)} dets)"
        )
        return m_pc

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