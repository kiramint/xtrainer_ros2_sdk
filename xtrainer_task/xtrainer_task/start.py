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
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image

from xtrainer_gripper.gripper_control import GripperController
from xtrainer_task.dino_wrapper import DinoWrapper
from xtrainer_task.robot_move import Planner, RobotMover

_COLOR_ENCODINGS = {"bgr8", "rgb8"}
_DEPTH_ENCODINGS = {"16UC1", "mono16"}

# 彩色 / 对齐深度话题模板
_COLOR_TOPIC_TEMPLATE = "/camera/{name}/color/image_raw"
_DEPTH_TOPIC_TEMPLATE = "/camera/{name}/aligned_depth_to_color/image_raw"

_CAMERA_NAMES = ("camera_top", "camera_left", "camera_right")

# 相机光学 frame 名称 (TODO: 填入 realsense 实际发布的 TF frame)
_CAMERA_OPTICAL_FRAMES: Dict[str, str] = {
    "camera_top": "camera_top_color_frame",      # TODO
    "camera_left": "camera_left_color_frame",    # TODO
    "camera_right": "camera_right_color_frame",  # TODO
}


class XTrainerTask(rclpy.Node):
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
        self.gripper = GripperController()

        # ── 彩色图缓存 (camera_name → latest BGR np.ndarray) ──
        self._color_frames: Dict[str, np.ndarray] = {}
        self._color_lock = threading.Lock()

        # ── 深度图缓存 (camera_name → latest aligned depth np.ndarray) ──
        self._depth_frames: Dict[str, np.ndarray] = {}
        self._depth_lock = threading.Lock()

        # ── 相机内参缓存 (camera_name → CameraInfo) ──
        self._camera_infos: Dict[str, CameraInfo] = {}
        self._camera_info_lock = threading.Lock()

        # ── TF2 缓冲与监听器 ──
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # 为三个相机分别订阅彩色、对齐深度 & 内参话题
        for name in _CAMERA_NAMES:
            color_topic = _COLOR_TOPIC_TEMPLATE.format(name=name)
            self.create_subscription(
                Image, color_topic,
                lambda msg, cam=name: self._color_callback(cam, msg),
                10,
            )
            self.get_logger().info(f"Subscribed color: {color_topic}")

            depth_topic = _DEPTH_TOPIC_TEMPLATE.format(name=name)
            self.create_subscription(
                Image, depth_topic,
                lambda msg, cam=name: self._depth_callback(cam, msg),
                10,
            )
            self.get_logger().info(f"Subscribed depth: {depth_topic}")

            info_topic = f"/camera/{name}/color/camera_info"
            self.create_subscription(
                CameraInfo, info_topic,
                lambda msg, cam=name: self._camera_info_callback(cam, msg),
                10,
            )
            self.get_logger().info(f"Subscribed camera_info: {info_topic}")

            # Launch Task
            self.create_timer(1.0,self.launch,oneshot=True)

    # ------------------------------------------------------------------
    # Controll Task ####################################################
    # ------------------------------------------------------------------

    def launch(self):
        """
        Step 1: Approach
        """
        image_top = self.get_latest_color("camera_top")
        
        result = self.dino.detect(image_top,"bottle")

        if len(result.scores) == 0:
            self.get_logger().warn("No bottle detected in top camera.")
            return
        
        annotated_image = self.dino.annotate(image_top,result,draw_mask=True)
        cv2.imshow("Detection Result", result.annotate(image_top, annotated_image))
        
        mid_x = (result.boxes[0]+result.boxes[3])/2
        mid_y = (result.boxes[1]+result.boxes[4])/2
        
        mid_coordinate = self.pixel_to_base_link("camera_top",mid_x,mid_y)

        rot = Rotation.from_euler('xyz',[0,0,0])

        # 10cm away from bottle
        pose_approach = Pose()
        pose_approach.position.x = mid_coordinate[0] - 0.1
        pose_approach.position.y = mid_coordinate[1]
        pose_approach.position.z = mid_coordinate[2]
        pose_approach.orientation.x = rot[0]
        pose_approach.orientation.y = rot[1]
        pose_approach.orientation.z = rot[2]
        pose_approach.orientation.w = rot[3]

        # move arm
        self.mover.plan_pose('Arm1',pose_approach,'L1_gripper_tcp',Planner.ompl)

        return 

        """
        Step 2: Grasp bottle
        """
        image_left = self.get_latest_color("camera_left")
        result = self.dino.detect(image_left,"bottle")

        mid_x = (result.boxes[0]+result.boxes[3])/2
        mid_y = (result.boxes[1]+result.boxes[4])/2
        
        mid_coordinate = self.pixel_to_base_link("camera_left",mid_x,mid_y)
        
        rot = Rotation.from_euler('xyz',[0,0,np.radians(-90)])

        # 10cm away from bottle
        pose_grasp = Pose()
        pose_grasp.position.x = mid_coordinate[0]
        pose_grasp.position.y = mid_coordinate[1]
        pose_grasp.position.z = mid_coordinate[2]
        pose_grasp.orientation.x = rot[0]
        pose_grasp.orientation.y = rot[1]
        pose_grasp.orientation.z = rot[2]
        pose_grasp.orientation.w = rot[3]

        # move arm
        self.mover.plan_pose('Arm1',pose_approach,'L1_gripper_tcp',Planner.pilz_ptp)

        # Grasp bottle
        self.gripper.set_torque(100) # TODO
        self.gripper.close("left", speed=1.0)

        # return # for test
        """
        Step 3: Grasp bottle cap
        """
        image_top = self.get_latest_color("camera_top")
        result = self.dino.detect(image_top,"bottle cap")
        
        mid_x = (result.boxes[0]+result.boxes[3])/2
        mid_y = (result.boxes[1]+result.boxes[4])/2
        
        mid_coordinate = self.pixel_to_base_link("camera_top",mid_x,mid_y)

        rot = Rotation.from_euler('xyz',[0,0,np.radians(-90)])

        # 10cm away from bottle
        pose_approach = Pose()
        pose_approach.position.x = mid_coordinate[0] + 0.1
        pose_approach.position.y = mid_coordinate[1]
        pose_approach.position.z = mid_coordinate[2]
        pose_approach.orientation.x = rot[0]
        pose_approach.orientation.y = rot[1]
        pose_approach.orientation.z = rot[2]
        pose_approach.orientation.w = rot[3]

        # move arm
        self.mover.plan_pose('Arm2',pose_approach,'L1_gripper_tcp',Planner.ompl)

        """
        Step 4: Grasp bottle cap
        """
        image_left = self.get_latest_color("camera_right")
        result = self.dino.detect(image_left,"white bottle cap")

        mid_x = (result.boxes[0]+result.boxes[3])/2
        mid_y = (result.boxes[1]+result.boxes[4])/2
        
        mid_coordinate = self.pixel_to_base_link("camera_right",mid_x,mid_y)
        
        rot = Rotation.from_euler('xyz',[0,0,np.radians(-90)])

        # 10cm away from bottle
        pose_grasp = Pose()
        pose_grasp.position.x = mid_coordinate[0]
        pose_grasp.position.y = mid_coordinate[1]
        pose_grasp.position.z = mid_coordinate[2]
        pose_grasp.orientation.x = rot[0]
        pose_grasp.orientation.y = rot[1]
        pose_grasp.orientation.z = rot[2]
        pose_grasp.orientation.w = rot[3]

        # move arm
        self.mover.plan_pose('Arm2',pose_approach,'L1_gripper_tcp',Planner.pilz_ptp)

        # Grasp bottle
        self.gripper.set_torque(100) # TODO
        self.gripper.close("right", speed=1.0)

        """
        Step 5: Open bottle cap
        """

        # TODO:


    # ------------------------------------------------------------------
    # 彩色回调
    # ------------------------------------------------------------------
    def _color_callback(self, camera_name: str, msg: Image) -> None:
        """将彩色帧转为 BGR 并缓存到 self._color_frames[camera_name]。"""
        try:
            color_bgr = self.ros_image_to_cv2(msg)
        except Exception as e:
            self.get_logger().error(
                f"[{camera_name}] color conversion failed: {e}",
                throttle_duration_sec=5.0,
            )
            return
        with self._color_lock:
            self._color_frames[camera_name] = color_bgr

    # ------------------------------------------------------------------
    # 彩色查询 API
    # ------------------------------------------------------------------
    def get_latest_color(self, camera_name: str) -> Optional[np.ndarray]:
        """返回最近一帧彩色图 (H×W×3 BGR uint8), 无缓存返回 None。"""
        with self._color_lock:
            return self._color_frames.get(camera_name)

    # ------------------------------------------------------------------
    # 深度回调
    # ------------------------------------------------------------------
    def _depth_callback(self, camera_name: str, msg: Image) -> None:
        """将对齐深度帧缓存到 self._depth_frames[camera_name]。"""
        try:
            depth = self._ros_depth_to_mm(msg)
        except Exception as e:
            self.get_logger().error(
                f"[{camera_name}] depth conversion failed: {e}",
                throttle_duration_sec=5.0,
            )
            return
        with self._depth_lock:
            self._depth_frames[camera_name] = depth

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
        """查询指定像素位置的对齐深度值 (物理尺度, 毫米)。

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
            深度值 (mm), 如果无缓 或无有效值则返回 None.
        """
        with self._depth_lock:
            depth = self._depth_frames.get(camera_name)

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
        """返回最近一帧对齐深度图 (mm, uint16), 无缓存返回 None。"""
        with self._depth_lock:
            return self._depth_frames.get(camera_name)

    # ------------------------------------------------------------------
    # 相机内参回调
    # ------------------------------------------------------------------
    def _camera_info_callback(self, camera_name: str, msg: CameraInfo) -> None:
        """缓存 camera_info (用于 pixel→3D 反投影)。"""
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