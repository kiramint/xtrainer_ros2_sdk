"""Entry point for xtrainer_task — robot movement using MoveIt.

Configs (URDF, SRDF, ompl_planning, etc.) are expected to be loaded
from ROS 2 parameters, which are set by the launch file:
    ros2 launch xtrainer_task start.launch.py

Usage
-----
    ros2 launch xtrainer_task start.launch.py
"""


import sys
import threading
import time
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
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
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
_FRESH_FRAME_TIMEOUT_SEC = 10.0


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

        # launch() 是长时间运行的阻塞任务。它不能与相机回调共用节点默认的
        # MutuallyExclusiveCallbackGroup，否则 launch() 开始后所有新图像回调都会饿死。
        self._camera_cb_group = ReentrantCallbackGroup()
        self._task_cb_group = MutuallyExclusiveCallbackGroup()

        # 保存订阅对象，明确维持其生命周期。
        self._camera_subscriptions = []

        # 为三个相机分别订阅彩色、对齐深度 & 内参话题
        for name in _CAMERA_NAMES:
            color_topic = _COLOR_TOPIC_TEMPLATE.format(name=name)
            color_sub = self.create_subscription(
                Image, color_topic,
                lambda msg, cam=name: self._color_callback(cam, msg),
                qos_profile_sensor_data,
                callback_group=self._camera_cb_group,
            )
            self._camera_subscriptions.append(color_sub)
            self.get_logger().info(f"Subscribed color: {color_topic}")

            depth_topic = _DEPTH_TOPIC_TEMPLATE.format(name=name)
            depth_sub = self.create_subscription(
                Image, depth_topic,
                lambda msg, cam=name: self._depth_callback(cam, msg),
                qos_profile_sensor_data,
                callback_group=self._camera_cb_group,
            )
            self._camera_subscriptions.append(depth_sub)
            self.get_logger().info(f"Subscribed depth: {depth_topic}")

            info_topic = f"/camera/{name}/color/camera_info"
            info_sub = self.create_subscription(
                CameraInfo, info_topic,
                lambda msg, cam=name: self._camera_info_callback(cam, msg),
                qos_profile_sensor_data,
                callback_group=self._camera_cb_group,
            )
            self._camera_subscriptions.append(info_sub)
            self.get_logger().info(f"Subscribed camera_info: {info_topic}")

        # ── 检测点 Marker 发布器 (RViz 可视化) ──
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._marker_pub = self.create_publisher(Marker, '/detection_marker', qos)

        self.get_logger().info('#################### All module initialized ######################')

        # Launch Task — 1s 后只执行一次 (Jazzy 已移除 oneshot 参数，回调内 cancel 替代)
        self._launch_timer = self.create_timer(
            1.0, self._on_launch_timer, callback_group=self._task_cb_group
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

        """
        Step 1: Approach
        """

        left_arm_z_axis = 0.0

        # Panning Loop
        while rclpy.ok():
            # Capture Loop
            while rclpy.ok():
                image_top = self.get_latest_color("camera_top")

                if image_top is None:
                    self.get_logger().warn(
                        "Waiting for top camera image...",
                        throttle_duration_sec=2.0,
                    )
                    time.sleep(0.1)
                    continue

                result = self.dino.detect(image_top, "bottle")

                if len(result.boxes) == 0:
                    self.get_logger().warn("No bottle detected in top camera.")
                    cv2.imshow("Detection Result", image_top)
                    cv2.waitKey(10)
                    continue

                try:
                    annotated_image = self.dino.annotate(image_top, result, draw_mask=True)
                except Exception:
                    self.get_logger().warn("Bottle annotate failed, showing original image")
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

            left_arm_z_axis = mid_coordinate[2]

            # 15cm away from bottle
            pose_approach = Pose()
            pose_approach.position.x = mid_coordinate[0] - 0.15
            pose_approach.position.y = mid_coordinate[1]
            pose_approach.position.z = mid_coordinate[2]
            pose_approach.orientation.x = -0.5022768378257751
            pose_approach.orientation.y = 0.4977162480354309
            pose_approach.orientation.z = -0.5023228526115417
            pose_approach.orientation.w = 0.4976627230644226

            self.get_logger().info(f"############# Move to pose {pose_approach} ###############")

            # move arm
            plan_result = self.mover.plan_pose('Arm1', pose_approach, planner=Planner.ompl)

            if plan_result is None:
                self.get_logger().error("################### Moveit Planning Failed #################")
                continue

            break

        if not self.mover.execute(plan_result.trajectory):
            self.get_logger().error("Step 1 trajectory execution failed; aborting task.")
            return

        # 按“本节点收到的帧序号”等待下一帧，不比较 ROS now() 与相机硬件时间戳。
        # RealSense 消息时间可能来自设备时钟，与节点时钟并非同一时间基准。
        step1_finish_sequence = self.get_color_sequence("camera_left")
        step1_finish_depth_sequence = self.get_depth_sequence("camera_left")

        """
        Step 2: Grasp bottle
        """
        fresh_frame_deadline = time.monotonic() + _FRESH_FRAME_TIMEOUT_SEC
        # Panning Loop
        while rclpy.ok():
            # Capture Loop — 等待 Arm1 移动后 camera_left 的新帧
            while rclpy.ok():
                image_left = self.get_latest_color("camera_left",
                                                   after_sequence=step1_finish_sequence)
                has_fresh_depth = (
                    self.get_depth_sequence("camera_left")
                    > step1_finish_depth_sequence
                )

                if image_left is None or not has_fresh_depth:
                    if time.monotonic() >= fresh_frame_deadline:
                        current_sequence = self.get_color_sequence("camera_left")
                        current_depth_sequence = self.get_depth_sequence("camera_left")
                        self.get_logger().error(
                            "Timed out waiting for fresh left camera color/depth "
                            f"(color {step1_finish_sequence}->{current_sequence}, "
                            f"depth {step1_finish_depth_sequence}->"
                            f"{current_depth_sequence}). Check camera topics and QoS."
                        )
                        return
                    self.get_logger().warn(
                        "Waiting for fresh left camera color/depth after arm movement...",
                        throttle_duration_sec=2.0,
                    )
                    time.sleep(0.1)
                    continue

                result = self.dino.detect(image_left, "biggest cylinder")

                if len(result.boxes) == 0:
                    self.get_logger().warn("No bottle detected in left camera.")
                    cv2.imshow("Detection Result", image_left)
                    cv2.waitKey(10)
                    continue

                try:
                    annotated_image = self.dino.annotate(image_left, result, draw_mask=True)
                except Exception:
                    self.get_logger().warn("Bottle annotate failed, showing original image")
                    annotated_image = image_left

                cv2.imshow("Detection Result", annotated_image)
                cv2.waitKey(10)

                break

            mid_x = (result.boxes[0, 0] + result.boxes[0, 2]) / 2
            mid_y = (result.boxes[0, 1] + result.boxes[0, 3]) / 2

            self.get_logger().info(f"Mid pixel: ({mid_x}, {mid_y})")

            mid_coordinate = self.pixel_to_base_link("camera_left", mid_x, mid_y)

            if mid_coordinate is None:
                self.get_logger().error("Failed to compute 3D coordinate of bottle center.")
                continue

            self._publish_detection_marker(mid_coordinate)

            # 15cm away from bottle
            pose_approach = Pose()
            pose_approach.position.x = mid_coordinate[0]
            pose_approach.position.y = mid_coordinate[1]
            pose_approach.position.z = left_arm_z_axis
            pose_approach.orientation.x = -0.5022768378257751
            pose_approach.orientation.y = 0.4977162480354309
            pose_approach.orientation.z = -0.5023228526115417
            pose_approach.orientation.w = 0.4976627230644226

            self.get_logger().info(f"############# Move to pose {pose_approach} ###############")

            # move arm
            plan_result = self.mover.plan_pose('Arm1', pose_approach, planner=Planner.pilz_ptp)

            if plan_result is None:
                self.get_logger().error("################### Moveit Planning Failed #################")
                continue

            break

        if not self.mover.execute(plan_result.trajectory):
            self.get_logger().error("Step 2 trajectory execution failed.")

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
        with self._snapshots_lock:
            snap = self._snapshots.setdefault(camera_name, {})
            snap['color'] = color_bgr
            snap['color_sequence'] = snap.get('color_sequence', 0) + 1
            snap['color_stamp'] = msg.header.stamp

    # ------------------------------------------------------------------
    # 彩色查询 API
    # ------------------------------------------------------------------
    def get_latest_color(
        self,
        camera_name: str,
        after_sequence: Optional[int] = None,
    ) -> Optional[np.ndarray]:
        """返回最近一帧彩色图 (H×W×3 BGR uint8).

        Parameters
        ----------
        camera_name : str
            相机名.
        after_sequence : int or None
            若提供，只返回序号严格大于该值的帧（用于可靠等待回调收到新帧）。

        Returns
        -------
        np.ndarray or None
            无快照或帧过旧时返回 None.
        """
        with self._snapshots_lock:
            snap = self._snapshots.get(camera_name)
            if snap is None or 'color' not in snap:
                return None
            if (after_sequence is not None
                    and snap.get('color_sequence', 0) <= after_sequence):
                return None
            return snap['color']

    def get_color_sequence(self, camera_name: str) -> int:
        """返回本节点已接收的彩色帧序号；尚未收到图像时返回 0。"""
        with self._snapshots_lock:
            snap = self._snapshots.get(camera_name)
            return snap.get('color_sequence', 0) if snap else 0

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
            snap['depth_sequence'] = snap.get('depth_sequence', 0) + 1
            snap['depth_stamp'] = msg.header.stamp
            snap['depth_frame_id'] = msg.header.frame_id

    def get_depth_sequence(self, camera_name: str) -> int:
        """返回本节点已接收的深度帧序号；尚未收到图像时返回 0。"""
        with self._snapshots_lock:
            snap = self._snapshots.get(camera_name)
            return snap.get('depth_sequence', 0) if snap else 0

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
        if not (0 <= u < w and 0 <= v < h):
            self.get_logger().warn(
                f"[{camera_name}] pixel ({u}, {v}) outside depth image {w}x{h}",
                throttle_duration_sec=2.0,
            )
            return None
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
        if fx == 0.0 or fy == 0.0:
            self.get_logger().error(f"[{camera_name}] invalid camera intrinsics")
            return None

        x_cam = (u - cx) / fx * depth_m
        y_cam = (v - cy) / fy * depth_m
        z_cam = depth_m

        # ── 4. tf2 变换到 base_link ──────────────────────────
        # CameraInfo 的 frame_id 是驱动实际使用的彩色光学坐标系，避免硬编码
        # camera_*_color_frame / camera_*_color_optical_frame 名称出错。
        optical_frame = info.header.frame_id
        if not optical_frame:
            self.get_logger().error(
                f"[{camera_name}] camera_info has an empty frame_id",
                throttle_duration_sec=10.0,
            )
            return None

        p_cam = PointStamped()
        p_cam.header.frame_id = optical_frame
        # Time(0) 请求最新可用 TF。对腕部相机使用 now() 容易因 TF 与图像发布
        # 存在几毫秒延迟而触发 future extrapolation。
        p_cam.header.stamp = rclpy.time.Time().to_msg()
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
        row_bytes = msg.width * 2
        if msg.step < row_bytes:
            raise ValueError(
                f"Unexpected step={msg.step}, expected at least {row_bytes}"
            )
        raw = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.step))
        packed = np.ascontiguousarray(raw[:, :row_bytes])
        dtype = np.dtype('>u2' if msg.is_bigendian else '<u2')
        return packed.view(dtype).reshape((msg.height, msg.width)).astype(
            np.uint16, copy=False
        )

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
        row_bytes = msg.width * 3
        if msg.step < row_bytes:
            raise ValueError(
                f"Unexpected step={msg.step}, expected at least {row_bytes}"
            )

        raw = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.step))
        frame = raw[:, :row_bytes].reshape((msg.height, msg.width, 3))
        if msg.encoding == "rgb8":
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        return np.ascontiguousarray(frame)

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