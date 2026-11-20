"""订阅三路彩色和对齐深度，按需编码成 JPEG / 16 位 PNG，并把像素变到 base_link。"""

from __future__ import annotations

import base64
import threading
import time
from typing import Optional

import numpy as np
import rclpy
import tf2_geometry_msgs  # noqa: F401  注册 PointStamped 的 tf2 转换
import tf2_ros
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image

from xtrainer_mcp.geometry import (
    CAMERAS,
    OPTICAL_FRAMES,
    f6,
    median_depth_mm,
    unproject,
)

_SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)
_STORE_INTERVAL = 0.1


def _stamp_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class CameraHub:
    def __init__(self, node: Node):
        self._node = node
        self._bridge = CvBridge()
        self._lock = threading.Lock()
        self._color: dict[str, dict] = {}
        self._depth: dict[str, dict] = {}
        self._info: dict[str, dict] = {}
        self._last_store: dict[str, float] = {}
        self._tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=30.0))
        self._tf_listener = tf2_ros.TransformListener(
            self._tf_buffer, node, spin_thread=False
        )
        for name in CAMERAS:
            node.create_subscription(
                Image,
                f"/camera/{name}/color/image_raw",
                lambda msg, camera=name: self._on_color(camera, msg),
                _SENSOR_QOS,
            )
            node.create_subscription(
                Image,
                f"/camera/{name}/aligned_depth_to_color/image_raw",
                lambda msg, camera=name: self._on_depth(camera, msg),
                _SENSOR_QOS,
            )
            node.create_subscription(
                CameraInfo,
                f"/camera/{name}/color/camera_info",
                lambda msg, camera=name: self._on_info(camera, msg),
                _SENSOR_QOS,
            )

    def _due(self, key: str) -> bool:
        now = time.monotonic()
        if now - self._last_store.get(key, 0.0) < _STORE_INTERVAL:
            return False
        self._last_store[key] = now
        return True

    def _on_color(self, camera: str, msg: Image) -> None:
        if not self._due(f"{camera}/color"):
            return
        try:
            image = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self._node.get_logger().warning(
                f"{camera} 彩色画面转换失败: {exc}", throttle_duration_sec=5.0
            )
            return
        with self._lock:
            self._color[camera] = {
                "image": np.array(image, copy=True),
                "stamp": msg.header.stamp,
                "frame_id": msg.header.frame_id,
            }

    def _on_depth(self, camera: str, msg: Image) -> None:
        if not self._due(f"{camera}/depth"):
            return
        try:
            image = self._bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as exc:
            self._node.get_logger().warning(
                f"{camera} 深度画面转换失败: {exc}", throttle_duration_sec=5.0
            )
            return
        depth = np.array(image, copy=True)
        if depth.dtype == np.float32:
            depth = np.clip(np.nan_to_num(depth) * 1000.0, 0, 65535).astype(np.uint16)
        elif depth.dtype != np.uint16:
            self._node.get_logger().warning(
                f"{camera} 深度编码 {msg.encoding} dtype={depth.dtype}，期望 16UC1",
                throttle_duration_sec=5.0,
            )
            return
        with self._lock:
            self._depth[camera] = {
                "image": depth,
                "stamp": msg.header.stamp,
                "frame_id": msg.header.frame_id or OPTICAL_FRAMES[camera],
                "encoding": msg.encoding,
            }

    def _on_info(self, camera: str, msg: CameraInfo) -> None:
        with self._lock:
            self._info[camera] = {
                "k": [float(v) for v in msg.k],
                "width": int(msg.width),
                "height": int(msg.height),
                "frame_id": msg.header.frame_id,
            }

    def _require_camera(self, camera: str) -> None:
        if camera not in OPTICAL_FRAMES:
            raise ValueError(
                f"camera 只能是 {', '.join(CAMERAS)}，收到 {camera!r}"
            )

    def get_frame(self, camera: str) -> dict:
        """有最新帧就立刻返回。没有帧不等待。"""
        self._require_camera(camera)
        import cv2

        with self._lock:
            color = self._color.get(camera)
            depth = self._depth.get(camera)
            info = self._info.get(camera)
        missing = []
        if color is None:
            missing.append("彩色")
        if depth is None:
            missing.append("深度")
        if missing:
            return {
                "ok": False,
                "error": f"{camera} 还没有{'/'.join(missing)}画面",
                "camera": camera,
            }
        color_ok, color_buf = cv2.imencode(
            ".jpg", color["image"], [int(cv2.IMWRITE_JPEG_QUALITY), 80]
        )
        depth_ok, depth_buf = cv2.imencode(".png", depth["image"])
        if not color_ok or not depth_ok:
            return {"ok": False, "error": f"{camera} 画面编码失败", "camera": camera}
        height, width = depth["image"].shape[:2]
        return {
            "ok": True,
            "camera": camera,
            "frame_id": depth["frame_id"],
            "width": int(width),
            "height": int(height),
            "color_stamp_sec": f6(_stamp_sec(color["stamp"])),
            "depth_stamp_sec": f6(_stamp_sec(depth["stamp"])),
            "k": None if info is None else info["k"],
            "depth_encoding": "16UC1",
            "depth_unit": "mm",
            "color_jpeg_base64": base64.b64encode(color_buf.tobytes()).decode("ascii"),
            "depth_png_base64": base64.b64encode(depth_buf.tobytes()).decode("ascii"),
        }

    def project_pixel(self, camera: str, u: int, v: int, window: int = 5) -> dict:
        self._require_camera(camera)
        with self._lock:
            depth = self._depth.get(camera)
            info = self._info.get(camera)
        if depth is None:
            return {"ok": False, "error": f"{camera} 还没有深度画面"}
        if info is None or len(info["k"]) != 9:
            return {"ok": False, "error": f"{camera} 还没有 color/camera_info"}
        depth_mm = median_depth_mm(depth["image"], int(u), int(v), int(window))
        if depth_mm is None:
            return {
                "ok": False,
                "error": f"{camera} 像素 ({int(u)}, {int(v)}) 窗口内没有有效深度",
            }
        fx, fy, cx, cy = info["k"][0], info["k"][4], info["k"][2], info["k"][5]
        optical = unproject(u, v, depth_mm / 1000.0, fx, fy, cx, cy)
        stamped = PointStamped()
        stamped.header.frame_id = depth["frame_id"]
        stamped.header.stamp = depth["stamp"]
        stamped.point.x, stamped.point.y, stamped.point.z = optical
        try:
            base = self._tf_buffer.transform(
                stamped,
                "base_link",
                timeout=rclpy.duration.Duration(seconds=1.0),
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
            tf2_ros.TransformException,
        ) as exc:
            return {
                "ok": False,
                "error": f"{camera} 的 TF 变换失败: {exc}",
                "optical_frame": depth["frame_id"],
                "optical_xyz": [f6(v) for v in optical],
                "depth_mm": round(depth_mm, 1),
                "stamp_sec": f6(_stamp_sec(depth["stamp"])),
            }
        return {
            "ok": True,
            "camera": camera,
            "pixel": {"u": int(u), "v": int(v)},
            "window": int(window),
            "depth_mm": round(depth_mm, 1),
            "optical_frame": depth["frame_id"],
            "optical_xyz": [f6(v) for v in optical],
            "base_frame": "base_link",
            "base_xyz": [f6(base.point.x), f6(base.point.y), f6(base.point.z)],
            "stamp_sec": f6(_stamp_sec(depth["stamp"])),
        }

    def transform_optical_point(self, camera: str, x: float, y: float, z: float) -> dict:
        """把光学坐标系里的一个点变到 base_link。用最新 TF。"""
        self._require_camera(camera)
        stamped = PointStamped()
        stamped.header.frame_id = OPTICAL_FRAMES[camera]
        stamped.header.stamp = rclpy.time.Time().to_msg()
        stamped.point.x = float(x)
        stamped.point.y = float(y)
        stamped.point.z = float(z)
        try:
            base = self._tf_buffer.transform(
                stamped,
                "base_link",
                timeout=rclpy.duration.Duration(seconds=1.0),
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
            tf2_ros.TransformException,
        ) as exc:
            return {"ok": False, "error": f"{camera} 的 TF 变换失败: {exc}"}
        return {
            "ok": True,
            "camera": camera,
            "optical_frame": OPTICAL_FRAMES[camera],
            "optical_xyz": [f6(x), f6(y), f6(z)],
            "base_frame": "base_link",
            "base_xyz": [f6(base.point.x), f6(base.point.y), f6(base.point.z)],
        }
