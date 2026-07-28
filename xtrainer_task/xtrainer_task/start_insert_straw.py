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
from collections import deque
from typing import Dict, Optional, Tuple

import cv2
import numpy as np
import rclpy
import tf2_geometry_msgs
import tf2_ros
from geometry_msgs.msg import PointStamped, Pose
from moveit.planning import MoveItPy
from rclpy.callback_groups import (MutuallyExclusiveCallbackGroup,
                                   ReentrantCallbackGroup)
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import CameraInfo, Image
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

_CAP_TURN_DEG = 180.0
_CAP_TURN_COUNT = 4
_CAP_GRIPPER_SETTLE_SEC = 1.0
_CAP_LIFT_DISTANCE_M = 0.05

# 瓶盖圆心跟踪：每轮超时后逐步放宽收敛阈值；全部轮次失败时，
# 缓存当前最佳估计并标记为低置信度，而不是让检测线程永久卡在旧状态。
_CAP_TRACK_BASE_STD_PX = 2.0
_CAP_TRACK_ROUND_TIMEOUT_SEC = 5.0
_CAP_TRACK_RELAX_FACTOR = 1.5
_CAP_TRACK_MAX_RELAX_ROUNDS = 3
_CAP_RESULT_MAX_AGE_SEC = 2.0

_STRAW_WRIST_SIGN = 1.0
_GRIPPER_AXIS_OFFSET_RAD = 0.0
_GRASP_HEIGHT_OFFSET_M = 0.025
_GRIPPER_AXIS_LOCAL = np.array([0.1, 0.0, 0.0], dtype=float)
_STRAW_AXIS_MIN_LENGTH_M = 0.015

_CAMERA_OPTICAL_FRAMES: Dict[str, str] = {
    "camera_top": "camera_top_color_optical_frame",
    "camera_left": "camera_left_color_optical_frame",
    "camera_right": "camera_right_color_optical_frame",
}


class CapCenterTracker:
    """多帧缓冲 + 离群点剔除 + 收敛判断。

    用法（每帧都调用 update，直到 is_converged() 返回 True 再取 get_stable_center()）：

        tracker = CapCenterTracker(buffer_size=15, converge_std_px=2.0)
        while True:
            frame = grab_frame()
            result = get_cap_center_hough(frame, bbox)
            tracker.update(result["center"], result["radius"])

            if tracker.is_converged():
                cx, cy = tracker.get_stable_center()
                r = tracker.get_stable_radius()
                break  # 拿到稳定结果，可以触发插入动作了
    """

    def __init__(self, buffer_size=15, converge_std_px=2.0,
                 mad_reject_thresh=3.0, min_samples_to_check=8):
        self.buffer_size = buffer_size
        self.converge_std_px = converge_std_px
        self.mad_reject_thresh = mad_reject_thresh
        self.min_samples_to_check = min_samples_to_check
        self.cx_buf = deque(maxlen=buffer_size)
        self.cy_buf = deque(maxlen=buffer_size)
        self.r_buf = deque(maxlen=buffer_size)
        self._lock = threading.RLock()

    def reset(self):
        """清空历史缓冲，避免跨位置的旧数据污染结果"""
        with self._lock:
            self.cx_buf.clear()
            self.cy_buf.clear()
            self.r_buf.clear()

    def set_converge_std_px(self, value):
        """更新收敛阈值（用于超时后的分轮降级）。"""
        with self._lock:
            self.converge_std_px = float(value)

    def update(self, center, radius):
        """center为None表示这一帧没检测到，直接跳过不计入缓冲"""
        if center is None or radius is None:
            return
        with self._lock:
            self.cx_buf.append(float(center[0]))
            self.cy_buf.append(float(center[1]))
            self.r_buf.append(float(radius))

    def _filtered_array(self, buf):
        """用中位数绝对偏差(MAD)剔除离群点，比直接算mean/std更抗单帧突跳干扰"""
        arr = np.array(buf)
        if len(arr) < 3:
            return arr
        med = np.median(arr)
        mad = np.median(np.abs(arr - med)) + 1e-6
        keep = np.abs(arr - med) / mad < self.mad_reject_thresh
        filtered = arr[keep]
        return filtered if len(filtered) > 0 else arr

    def is_converged(self):
        """判断当前缓冲区是否已经收敛到稳定值"""
        with self._lock:
            if len(self.cx_buf) < self.min_samples_to_check:
                return False
            cx_f = self._filtered_array(self.cx_buf)
            cy_f = self._filtered_array(self.cy_buf)
            return (
                cx_f.std() < self.converge_std_px
                and cy_f.std() < self.converge_std_px
            )

    def get_stable_center(self):
        """取剔除离群点后的中位数作为最终稳定坐标"""
        with self._lock:
            if not self.cx_buf:
                return None
            cx_f = self._filtered_array(self.cx_buf)
            cy_f = self._filtered_array(self.cy_buf)
            return float(np.median(cx_f)), float(np.median(cy_f))

    def get_stable_radius(self):
        with self._lock:
            if not self.r_buf:
                return None
            r_f = self._filtered_array(self.r_buf)
            return float(np.median(r_f))

    def get_current_std(self):
        """调试用：查看当前抖动幅度"""
        with self._lock:
            if len(self.cx_buf) < 2:
                return None, None
            cx_f = self._filtered_array(self.cx_buf)
            cy_f = self._filtered_array(self.cy_buf)
            return float(cx_f.std()), float(cy_f.std())

    def get_sample_count(self):
        with self._lock:
            return len(self.cx_buf)

    def get_detection_rate(self, total_frames_seen):
        if total_frames_seen <= 0:
            return 0.0
        return self.get_sample_count() / total_frames_seen

    def is_bimodal(self, gap_ratio_thresh=3.0):
        """粗略检测误检导致的双峰/多峰样本分布。"""
        with self._lock:
            if len(self.cx_buf) < self.min_samples_to_check:
                return False
            for buf in (self.cx_buf, self.cy_buf):
                arr = np.sort(np.asarray(buf, dtype=float))
                diffs = np.diff(arr)
                if not len(diffs):
                    continue
                max_gap = float(diffs.max())
                median_gap = float(np.median(diffs)) + 1e-6
                if (
                    max_gap > gap_ratio_thresh * median_gap
                    and max_gap > 5.0
                ):
                    return True
            return False


class XTrainerTask(Node):
    def __init__(self):
        super().__init__("xtrainer_task")

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

        # ── 各相机最新帧快照 (单锁, 每帧覆盖) ──
        self._snapshots: Dict[str, dict] = {}
        self._snapshots_lock = threading.Lock()

        # ── 相机内参缓存 (camera_name → CameraInfo) ──
        self._camera_infos: Dict[str, CameraInfo] = {}
        self._camera_info_lock = threading.Lock()

        # ── TF2 缓冲与监听器 ──
        self._tf_buffer = tf2_ros.Buffer()
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

        # ── 检测点 Marker 发布器 (RViz 可视化) ──
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._marker_pub = self.create_publisher(Marker, '/detection_marker', qos)

        self.get_logger().info('#################### All module initialized ######################')

        # Launch Task — 1s 后只执行一次 (Jazzy 已移除 oneshot 参数，回调内 cancel 替代)
        # ── DINO 互斥锁 ──
        # DINO/SAM2 使用 CUDA，多个线程同时调用会导致 GPU 竞争闪退。
        # 后台瓶盖检测线程与 launch() 主流程共享此锁。
        self._dino_lock = threading.Lock()

        # ── 瓶盖检测后台线程 ──
        # CapCenterTracker 负责多帧收敛判断，从 init 开始持续运行
        self._cap_tracker = CapCenterTracker(
            buffer_size=15, converge_std_px=_CAP_TRACK_BASE_STD_PX,
        )
        self._cap_cache: Dict[str, object] = {
            "center": None,
            "radius": None,
            "confident": False,
            "status": "starting",
            "updated_monotonic": 0.0,
            "round": 0,
            "std_threshold_px": _CAP_TRACK_BASE_STD_PX,
            "detection_rate": 0.0,
        }
        self._cap_cache_lock = threading.Lock()
        self._cap_vis_img: Optional[np.ndarray] = None  # 后台线程缓存的可视化图
        self._cap_vis_lock = threading.Lock()
        self._cap_thread_running = True
        self._cap_thread = threading.Thread(
            target=self._cap_detection_loop, daemon=True,
        )
        self._cap_thread.start()
        self.get_logger().info("Cap detection background thread started.")

        self._launch_timer = self.create_timer(
            1.0,
            self._on_launch_timer,
            callback_group=self._launch_cb_group
        )

    def _update_cap_cache(self, **values) -> None:
        """原子更新后台瓶盖检测结果。"""
        with self._cap_cache_lock:
            self._cap_cache.update(values)

    def _get_fresh_cap_result(self) -> Optional[Dict[str, object]]:
        """返回仍在有效期内的圆心结果，防止 Step 4 使用陈旧缓存。"""
        with self._cap_cache_lock:
            cached = dict(self._cap_cache)
        if cached.get("center") is None or cached.get("radius") is None:
            return None
        updated = float(cached.get("updated_monotonic", 0.0))
        cached["age_sec"] = time.monotonic() - updated
        if cached["age_sec"] > _CAP_RESULT_MAX_AGE_SEC:
            return None
        return cached

    # ------------------------------------------------------------------
    # 瓶盖检测后台线程 — 从 init 后持续运行，缓存检测结果
    # ------------------------------------------------------------------
    def _cap_detection_loop(self):
        """持续检测瓶盖，并通过分轮降级策略维护新鲜结果缓存。"""
        self.get_logger().info("Cap detection loop started.")

        round_idx = 0
        round_started = time.monotonic()
        total_frames_seen = 0
        last_stamp_ns = -1
        self._cap_tracker.reset()
        self._cap_tracker.set_converge_std_px(_CAP_TRACK_BASE_STD_PX)

        while self._cap_thread_running and rclpy.ok():
            # 只处理新帧，避免对同一张图重复推理并错误抬高成功率。
            with self._snapshots_lock:
                snap = self._snapshots.get("camera_top")
                image = snap.get("color") if snap else None
                stamp_ns = snap.get("stamp_ns", -1) if snap else -1
            if image is None:
                time.sleep(0.05)
                continue
            if stamp_ns == last_stamp_ns:
                time.sleep(0.01)
                continue
            last_stamp_ns = stamp_ns
            total_frames_seen += 1

            std_threshold = (
                _CAP_TRACK_BASE_STD_PX
                * (_CAP_TRACK_RELAX_FACTOR ** round_idx)
            )
            vis = image.copy()
            valid_detection = False

            try:
                # DINO/SAM2 不是线程安全的，必须与 Step 1/2 串行使用。
                with self._dino_lock:
                    result = self.dino.detect(image, "bottle")
                if len(result.boxes) > 0:
                    circle_result = self.get_cap_center_hough(
                        image, result.boxes[0, :4], draw=True,
                    )
                    vis = circle_result["vis_img"]
                    center = circle_result["center"]
                    radius = circle_result["radius"]
                    if center is not None and radius is not None:
                        self._cap_tracker.update(center, radius)
                        valid_detection = True
                else:
                    cv2.putText(
                        vis, "NO BOTTLE DETECTED", (10, 90),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2,
                    )
            except Exception as exc:
                # 单帧失败不能杀死后台线程；下一张新帧自动继续。
                self.get_logger().warn(
                    f"Cap detection frame failed: {exc}",
                    throttle_duration_sec=2.0,
                )
                cv2.putText(
                    vis, "DETECTION ERROR - RETRYING", (10, 90),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2,
                )

            detection_rate = self._cap_tracker.get_detection_rate(
                total_frames_seen
            )
            elapsed = time.monotonic() - round_started

            if valid_detection and self._cap_tracker.is_converged():
                stable_center = self._cap_tracker.get_stable_center()
                stable_radius = self._cap_tracker.get_stable_radius()
                self._update_cap_cache(
                    center=stable_center,
                    radius=stable_radius,
                    confident=True,
                    status="converged",
                    updated_monotonic=time.monotonic(),
                    round=round_idx,
                    std_threshold_px=std_threshold,
                    detection_rate=detection_rate,
                )
                cv2.putText(
                    vis, "CONVERGED", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2,
                )
                cv2.putText(
                    vis,
                    f"stable=({stable_center[0]:.1f}, "
                    f"{stable_center[1]:.1f})",
                    (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (0, 255, 0), 1,
                )
                # 持续稳定检测属于健康状态；只有连续一个完整窗口未能
                # 再次收敛时，才进入阈值放宽流程。
                round_started = time.monotonic()
                elapsed = 0.0
            else:
                std_x, std_y = self._cap_tracker.get_current_std()
                self._update_cap_cache(
                    status="tracking",
                    round=round_idx,
                    std_threshold_px=std_threshold,
                    detection_rate=detection_rate,
                )
                cv2.putText(
                    vis,
                    f"TRACKING round={round_idx} "
                    f"threshold={std_threshold:.1f}px",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (0, 165, 255), 2,
                )
                if std_x is not None:
                    cv2.putText(
                        vis,
                        f"std=({std_x:.1f},{std_y:.1f}) "
                        f"rate={detection_rate:.0%}",
                        (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                        (0, 165, 255), 1,
                    )

            # 本轮超时：放宽阈值并清空样本；最后一轮发布当前最佳
            # 低置信度估计，然后自动从严格阈值开启下一周期。
            if elapsed >= _CAP_TRACK_ROUND_TIMEOUT_SEC:
                sample_count = self._cap_tracker.get_sample_count()
                std_x, std_y = self._cap_tracker.get_current_std()
                bimodal = self._cap_tracker.is_bimodal()
                self.get_logger().warn(
                    f"Cap tracking round {round_idx} timed out: "
                    f"samples={sample_count}/{total_frames_seen}, "
                    f"rate={detection_rate:.1%}, std=({std_x}, {std_y}), "
                    f"bimodal={bimodal}, threshold={std_threshold:.1f}px"
                )

                if round_idx < _CAP_TRACK_MAX_RELAX_ROUNDS:
                    round_idx += 1
                    next_threshold = (
                        _CAP_TRACK_BASE_STD_PX
                        * (_CAP_TRACK_RELAX_FACTOR ** round_idx)
                    )
                    self.get_logger().warn(
                        f"Cap tracking retry round {round_idx}; relaxing "
                        f"threshold to {next_threshold:.1f}px"
                    )
                else:
                    fallback_center = self._cap_tracker.get_stable_center()
                    fallback_radius = self._cap_tracker.get_stable_radius()
                    if fallback_center is not None and fallback_radius is not None:
                        self._update_cap_cache(
                            center=fallback_center,
                            radius=fallback_radius,
                            confident=False,
                            status="degraded",
                            updated_monotonic=time.monotonic(),
                            round=round_idx,
                            std_threshold_px=std_threshold,
                            detection_rate=detection_rate,
                        )
                        self.get_logger().warn(
                            "Cap tracking degraded result available: "
                            f"center=({fallback_center[0]:.1f}, "
                            f"{fallback_center[1]:.1f}), "
                            f"radius={fallback_radius:.1f}px"
                        )
                    else:
                        self._update_cap_cache(
                            center=None,
                            radius=None,
                            confident=False,
                            status="failed_no_detection",
                            updated_monotonic=0.0,
                        )
                        self.get_logger().error(
                            "Cap tracking cycle failed: no valid circle "
                            "was detected; restarting automatically"
                        )
                    round_idx = 0

                self._cap_tracker.reset()
                self._cap_tracker.set_converge_std_px(
                    _CAP_TRACK_BASE_STD_PX
                    * (_CAP_TRACK_RELAX_FACTOR ** round_idx)
                )
                round_started = time.monotonic()
                total_frames_seen = 0

            with self._cap_vis_lock:
                self._cap_vis_img = vis

            time.sleep(0.01)

        self.get_logger().info("Cap detection loop stopped.")

    # ------------------------------------------------------------------
    # Controll Task ####################################################
    # ------------------------------------------------------------------

    def _on_launch_timer(self):
        """One-shot timer callback — cancels timer after first invocation."""
        self._launch_timer.cancel()
        self.launch()

    def launch(self):

        grasp_tip = "L1_gripper_tip"

        self.gripper.open_both()

        """
        Step 1: Approach
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

                with self._dino_lock:
                    result = self.dino.detect(image_top, "straw")

                if len(result.boxes) == 0:
                    self.get_logger().warn("No straw detected in top camera.")
                    cv2.imshow("Detection Result", image_top)
                    cv2.waitKey(1)
                    time.sleep(0.05)
                    continue

                try:
                    annotated_image = self.dino.annotate(image_top, result, draw_mask=True)
                except Exception:
                    self.get_logger().warn("Straw annotate failed, showing original image")
                    annotated_image = image_top

                cv2.imshow("Detection Result", annotated_image)
                cv2.waitKey(10)

                break

            mid_x = (result.boxes[0, 0] + result.boxes[0, 2]) / 2
            mid_y = (result.boxes[0, 1] + result.boxes[0, 3]) / 2

            self.get_logger().info(f"Mid pixel: ({mid_x}, {mid_y})")

            mid_coordinate = self.pixel_to_base_link("camera_top", mid_x, mid_y)

            if mid_coordinate is None:
                self.get_logger().error("Failed to compute 3D coordinate of straw center.")
                continue

            self._publish_detection_marker(mid_coordinate)

            pose_prepare = Pose()
            pose_prepare.position.x = mid_coordinate[0]
            pose_prepare.position.y = mid_coordinate[1]
            pose_prepare.position.z = mid_coordinate[2] + 0.2
            pose_prepare.orientation.x = 0.999996542930603
            pose_prepare.orientation.y = -6.24120730208233e-07
            pose_prepare.orientation.z = 0.0026403707452118397
            pose_prepare.orientation.w = 3.604810103752243e-07

            self.get_logger().info(f"############# Move to pose {pose_prepare} ###############")

            # move arm
            plan_result = self.mover.plan_pose('Arm1', pose_prepare, planner=Planner.ompl,tip_link=grasp_tip)

            if plan_result is None:
                self.get_logger().error("################### Moveit Planning Failed #################")
                continue

            break

        self.mover.execute(plan_result.trajectory)

        # 等待关节状态稳定，避免 Step 1 执行后 MoveIt 内部状态
        # 与真实 /joint_states 尚未同步，导致下一步 get_current_pose
        # 读到的起点与物理状态偏差超过 allowed_start_tolerance (0.01)。
        time.sleep(0.5)

        """
        Step 2: Grasp Straw
        """

        # Panning Loop
        while rclpy.ok():
            # Sub Step1: Capture Loop
            while rclpy.ok():
                image_left = self.get_latest_color("camera_left")

                if image_left is None:
                    self.get_logger().error("No left camera image available.")
                    time.sleep(0.05)
                    continue

                with self._dino_lock:
                    result = self.dino.detect(image_left, "white pipe")

                if len(result.boxes) == 0:
                    self.get_logger().warn("No straw detected in left camera.")
                    cv2.imshow("Detection Result", image_left)
                    cv2.waitKey(1)
                    time.sleep(0.05)
                    continue

                try:
                    annotated_image = self.dino.annotate(image_left, result, draw_mask=True)
                except Exception:
                    self.get_logger().warn("Straw annotate failed, showing original image")
                    annotated_image = image_left

                cv2.imshow("Detection Result", annotated_image)
                cv2.waitKey(10)

                break

            # SubStep: 2 Get Axis
            # 用 SAM2 mask 的主轴取吸管两端，而不是使用 box 对角线。
            # 对斜拍相机，两个端点各自取深度并经完整 TF 变换到 base_link，
            # 再计算真实 3D 方向；这避免直接把图像角度当成 J1_6 角度。
            straw_axis = self._estimate_straw_axis_base(
                "camera_left", image_left, result
            )
            if straw_axis is None:
                self.get_logger().warn(
                    "Cannot estimate a valid 3D straw axis; retrying detection."
                )
                time.sleep(0.05)
                continue

            straw_center, straw_direction = straw_axis      # 吸管中心与方向向量
            straw_direction_xy = np.asarray(straw_direction[:2], dtype=float)   # 提取方向向量的XY平面分量（忽略Z轴）
            straw_length_xy = float(np.linalg.norm(straw_direction_xy))
            if straw_length_xy < _STRAW_AXIS_MIN_LENGTH_M:
                self.get_logger().warn(
                    "Straw 3D axis has insufficient XY projection; "
                    "check camera calibration or depth data."
                )
                continue
            straw_direction_xy /= straw_length_xy   # 归一化XY平面方向向量（单位向量）

            # SubStep: 3 Grasp Pose
            # 夹爪 x 轴（手指开合方向）需垂直吸管，从侧面抓取
            # → yaw = 吸管角度 + 90°
            # 轴没有正负方向：θ 与 θ±π 对对称夹爪等价，归一化到 [-π/2, π/2]
            straw_yaw = math.atan2(straw_direction_xy[1], straw_direction_xy[0])
            if straw_yaw > math.pi / 2:
                straw_yaw -= math.pi
            elif straw_yaw <= -math.pi / 2:
                straw_yaw += math.pi
            straw_yaw = _STRAW_WRIST_SIGN * (
                straw_yaw + _GRIPPER_AXIS_OFFSET_RAD
            )
            grasp_yaw = straw_yaw - math.pi / 2

            self.get_logger().info(
                f"Straw center=({straw_center[0]:.3f}, {straw_center[1]:.3f}, "+
                f"{straw_center[2]:.3f}), "+
                f"axis=({straw_direction[0]:.3f}, {straw_direction[1]:.3f}, "+
                f"{straw_direction[2]:.3f}), "+
                f"straw yaw={math.degrees(straw_yaw):.1f} deg, "+
                f"grasp yaw={math.degrees(grasp_yaw):.1f} deg"
            )

            roll = -np.pi
            pitch = 0.0
            yaw = grasp_yaw

            rotation = R.from_euler("xyz",[roll,pitch,yaw])
            quat = rotation.as_quat()

            grasp_pose = self.mover.get_current_pose(
                "Arm1", tip_link=grasp_tip
            )

            grasp_pose.position.x = straw_center[0]
            grasp_pose.position.y = straw_center[1]
            grasp_pose.position.z = straw_center[2]
            grasp_pose.orientation.x = quat[0]
            grasp_pose.orientation.y = quat[1]
            grasp_pose.orientation.z = quat[2]
            grasp_pose.orientation.w = quat[3]

            plan_grasp = self.mover.plan_pose(
                "Arm1", grasp_pose, planner=Planner.pilz_ptp, tip_link=grasp_tip,
            )
            if plan_grasp is None or not self.mover.execute(plan_grasp.trajectory):
                self.get_logger().error("Failed to descend to straw for grasping.")
                continue

            self.gripper.close("left", speed=0.35)
            time.sleep(_CAP_GRIPPER_SETTLE_SEC)
            status = self.gripper.read_status("left")
            if status not in ("CLOSED", "MOVING"):
                self.get_logger().warn(
                    f"Left gripper status after grasp: {status}; "
                    "grasp may have failed."
                )
            self.get_logger().info("Step 2 completed: straw grasp command sent.")
            
            break

        # Step 2 执行 + 夹爪闭合后关节状态可能有所偏移，
        # 稍等一下让 /joint_states 更新到 MoveIt planning scene。
        time.sleep(0.5)

        """
        Step 3: Move to wait pose
        """
        while rclpy.ok():
            wait_pose = self.mover.get_current_pose(
                            "Arm1", tip_link=grasp_tip
                        )
            wait_pose.position.z = wait_pose.position.z + 0.2

            plan_grasp = self.mover.plan_pose(
                "Arm1", wait_pose, planner=Planner.pilz_ptp,
                tip_link=grasp_tip,
            )

            if plan_grasp is None or not self.mover.execute(plan_grasp.trajectory):
                self.get_logger().error("Failed to move to wait pose.")
                continue

            break

        """
        Step 4: Find bottle and move
        """
        # 后台线程自 init 起持续检测瓶盖圆心并缓存结果。
        # 此处等待 tracker 收敛后读取稳定圆心，再规划插入路径。
        self.get_logger().info(
            "Step 4: waiting for cap center to converge ..."
        )

        while rclpy.ok():

            # ── 等待后台给出“新鲜”的收敛或降级结果 ──
            # 后台每 5 秒自动放宽一次阈值，4 轮后最多约 20 秒会给出
            # confident=False 的最佳估计，因此这里保留 30 秒总保护超时。
            cap_wait_deadline = time.monotonic() + 30.0
            cap_result = None
            while rclpy.ok() and time.monotonic() < cap_wait_deadline:
                # 显示后台线程缓存的检测可视化
                with self._cap_vis_lock:
                    vis = self._cap_vis_img
                if vis is not None:
                    cv2.imshow("Cap Detection", vis)
                    cv2.waitKey(1)

                cap_result = self._get_fresh_cap_result()
                if cap_result is not None:
                    break

                with self._cap_cache_lock:
                    status = dict(self._cap_cache)
                self.get_logger().info(
                    "Waiting for fresh cap result: "
                    f"status={status.get('status')}, "
                    f"round={status.get('round')}, "
                    f"threshold={status.get('std_threshold_px')}px, "
                    f"rate={float(status.get('detection_rate', 0.0)):.1%}",
                    throttle_duration_sec=2.0,
                )
                time.sleep(0.1)
            else:
                self.get_logger().error(
                    "No fresh cap center became available within 30 seconds. "
                    "Check the bottle bbox, lighting, and Hough parameters."
                )
                return  # 终止任务

            # ── 读取后台缓存；不再直接读 tracker，避免跨轮 reset 竞态 ──
            cx, cy = cap_result["center"]
            r_stable = float(cap_result["radius"])
            if cap_result["confident"]:
                self.get_logger().info(
                    f"Cap center converged: ({cx:.1f}, {cy:.1f}) px, "
                    f"radius={r_stable:.1f}px, "
                    f"round={cap_result['round']}, "
                    f"age={cap_result['age_sec']:.2f}s"
                )
            else:
                self.get_logger().warn(
                    "Using timeout-degraded cap estimate: "
                    f"center=({cx:.1f}, {cy:.1f})px, "
                    f"radius={r_stable:.1f}px, "
                    f"rate={float(cap_result['detection_rate']):.1%}, "
                    f"age={cap_result['age_sec']:.2f}s. "
                    "Insert slowly or add a task-level confirmation step."
                )

            # ── 抓一帧地面实况叠加稳定圆，确认最终结果 ──
            image_top = self.get_latest_color("camera_top")
            if image_top is not None:
                vis_img = image_top.copy()
                cx_i, cy_i = int(round(cx)), int(round(cy))
                r_i = int(round(r_stable))
                cv2.circle(vis_img, (cx_i, cy_i), r_i, (0, 255, 0), 3)
                cv2.drawMarker(vis_img, (cx_i, cy_i), (0, 255, 0),
                               cv2.MARKER_CROSS, 30, 3)
                label = f"STABLE ({cx_i}, {cy_i})  r={r_i}"
                cv2.putText(vis_img, label, (cx_i - 80, cy_i - r_i - 15),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                cv2.imshow("Cap Detection", vis_img)
                cv2.waitKey(1)

            # ── 像素 → base_link 3D 坐标 ──
            cap_3d = self.pixel_to_base_link("camera_top", int(cx), int(cy))
            if cap_3d is None:
                self.get_logger().error(
                    "Failed to compute 3D coordinate of cap center."
                )
                return

            self._publish_detection_marker(cap_3d)
            self.get_logger().info(
                f"Cap center in base_link: ({cap_3d[0]:.4f}, "
                f"{cap_3d[1]:.4f}, {cap_3d[2]:.4f})"
            )

            insert_pose = Pose()
            insert_pose.position.x = cap_3d[0]
            insert_pose.position.y = cap_3d[1]
            insert_pose.position.z = cap_3d[2] + 0.3
            insert_pose.orientation.x = -0.5022768378257751
            insert_pose.orientation.y = 0.4977162480354309
            insert_pose.orientation.z = -0.5023228526115417
            insert_pose.orientation.w = 0.4976627230644226

            grasp_tip = "L1_gripper_tip"
            plan_insert = self.mover.plan_pose(
                "Arm1", insert_pose, planner=Planner.ompl, tip_link=grasp_tip,
            )

            if plan_insert is None or not self.mover.execute(plan_insert.trajectory):
                self.get_logger().error("Failed to move to insert pose.")
                continue

            break

        """
        Step 5: Stick into it
        """

        while rclpy.ok():
            stick_pose = insert_pose
            stick_pose.position.z = cap_3d[2] + 0.05

            grasp_tip = "L1_gripper_tip"
            plan_insert = self.mover.plan_pose(
                "Arm1", insert_pose, planner=Planner.pilz_ptp, tip_link=grasp_tip,
            )

            if plan_insert is None or not self.mover.execute(plan_insert.trajectory):
                self.get_logger().error("Failed to move to stck pose.")
                continue

            break

        self.gripper.open_both()



    @staticmethod
    def get_cap_center_hough(img_bgr, bbox, minRadius=15, maxRadius=70,
                             param1=50, param2=25, draw=True):
        """
        在给定bbox范围内检测圆形瓶盖中心。
    
        Args:
            img_bgr: 原图 (BGR)
            bbox: (x1, y1, x2, y2)，GroundingDINO等给出的粗定位框
            minRadius, maxRadius: 圆半径搜索范围（像素），需要根据实际瓶盖尺寸/相机高度标定
            param1: Canny边缘检测高阈值
            param2: 累加器阈值，越小越容易检出（也越容易误检），需要调参
            draw: 是否返回可视化标注图
    
        Returns:
            result: dict，包含:
                - center: (cx, cy) 原图坐标系下的圆心，若未检测到则为 None
                - radius: 半径（像素），若未检测到则为 None
                - vis_img: 标注后的图像（仅当 draw=True 且检测成功时返回，否则为原图副本）
        """
        x1, y1, x2, y2 = bbox
        x1, y1, x2, y2 = int(x1),int(y1),int(x2),int(y2)
        roi = img_bgr[y1:y2, x1:x2]
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        enhanced = clahe.apply(gray)
        denoised = cv2.bilateralFilter(enhanced, 9, 75, 75)  # 去噪不是锐化
    
        circles = cv2.HoughCircles(
            denoised, cv2.HOUGH_GRADIENT, dp=1, minDist=50,
            param1=param1, param2=param2,
            minRadius=minRadius, maxRadius=maxRadius
        )
    
        vis_img = img_bgr.copy()
        # 始终画出ROI框，方便调参时确认bbox是否框对了
        cv2.rectangle(vis_img, (x1, y1), (x2, y2), (255, 0, 0), 1)
    
        if circles is None:
            cv2.putText(vis_img, "NO CIRCLE DETECTED", (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            return {"center": None, "radius": None, "vis_img": vis_img}
    
        # circles[0] 已按累加器得分排序，取第一个作为最佳检测结果
        cx_roi, cy_roi, r = circles[0][0]
        center_full = (float(cx_roi + x1), float(cy_roi + y1))
    
        if draw:
            cx_i, cy_i = int(round(center_full[0])), int(round(center_full[1]))
            r_i = int(round(r))
            # 圆轮廓
            cv2.circle(vis_img, (cx_i, cy_i), r_i, (0, 0, 255), 2)
            # 圆心十字标记
            cv2.drawMarker(vis_img, (cx_i, cy_i), (0, 255, 255),
                            cv2.MARKER_CROSS, 20, 2)
            # 坐标文本标注
            label = f"({cx_i}, {cy_i})  r={r_i}"
            cv2.putText(vis_img, label, (cx_i - 60, cy_i - r_i - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
    
        return {"center": center_full, "radius": float(r), "vis_img": vis_img}

    def _estimate_straw_axis_base(
        self, camera_name: str, image: np.ndarray, result
    ) -> Optional[Tuple[Tuple[float, float, float], Tuple[float, float, float]]]:
        """从 mask PCA 选两端点，并将其 3D 坐标转换到 base_link。"""
        if result.masks is None or len(result.masks) == 0:
            return None
        mask = np.asarray(result.masks[0], dtype=bool)
        ys, xs = np.nonzero(mask)
        if len(xs) < 20:
            return None
        points = np.column_stack((xs.astype(float), ys.astype(float)))
        _, _, vh = np.linalg.svd(points - points.mean(axis=0), full_matrices=False)
        principal = vh[0]
        projection = (points - points.mean(axis=0)) @ principal # 将所有点投影到主方向上，得到一维投影值
        low = points[np.argmin(projection)]     # 找到投影值最小的点（轴的一端）
        high = points[np.argmax(projection)]    # 找到投影值最大的点（轴的另一端）
        p0 = self.pixel_to_base_link(camera_name, int(low[0]), int(low[1]), window=3)
        p1 = self.pixel_to_base_link(camera_name, int(high[0]), int(high[1]), window=3)
        if p0 is None or p1 is None:
            return None
        direction = np.asarray(p1, dtype=float) - np.asarray(p0, dtype=float) # 计算从p0到p1的方向向量
        length = float(np.linalg.norm(direction))
        if length < _STRAW_AXIS_MIN_LENGTH_M:
            return None
        center = self.pixel_to_base_link(camera_name, int((low[0]+high[0])/2), int((low[1]+high[1])/2), window=3)
        return center, tuple((direction / length).tolist())

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