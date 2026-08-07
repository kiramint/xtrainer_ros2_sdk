#!/usr/bin/env python3
"""GroundingDINO + 矩形拟合方向估计测试。

订阅相机彩色图像，用 GroundingDINO 检测 ``bottle``，
对每个检测到的 bottle mask 做最小外接矩形拟合 (``cv2.minAreaRect``)，
得到物体较长边的方向向量并绘制箭头显示。

.. note::

    此脚本不依赖 ``cv_bridge``，直接解析 ``sensor_msgs/Image`` 原始数据。

Usage::

    export PYTHONPATH=/opt/Project/Grounded-SAM-2/grounding_dino:/opt/Project/Grounded-SAM-2:$PYTHONPATH
    ros2 run xtrainer_task dino_pose_test --ros-args -p image_topic:="/camera/camera_top/color/image_raw"
"""

import math
import sys
from typing import Optional, Tuple

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from xtrainer_task.dino_wrapper import DinoWrapper

_SUPPORTED_ENCODINGS = {"bgr8", "rgb8"}


def ros_image_to_cv2(msg: Image) -> np.ndarray:
    """将 ``sensor_msgs/Image`` 转换为 OpenCV BGR 图像 (不依赖 cv_bridge)。"""
    if msg.encoding not in _SUPPORTED_ENCODINGS:
        raise ValueError(
            f"Unsupported encoding: '{msg.encoding}', "
            f"expected one of {_SUPPORTED_ENCODINGS}"
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


def fit_rectangle_orientation(mask: np.ndarray):
    """对二值 mask 做最小外接矩形拟合，返回物体较长边的方向向量。

    使用 ``cv2.minAreaRect`` 对 mask 的所有前景像素拟合最小面积外接矩形，
    再由矩形 4 个角点计算两条邻边向量，取较长者作为物体主轴方向。

    Parameters
    ----------
    mask : np.ndarray
        ``(H, W)`` bool 数组，True 为物体像素。

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
        "axis": axis.astype(np.float32),
        "length": long_len,
        "angle_deg": angle_deg,
    }


def draw_orientation(
    image: np.ndarray,
    fit: dict,
    color_rect: Tuple[int, int, int] = (0, 255, 0),
    color_axis: Tuple[int, int, int] = (0, 0, 255),
) -> np.ndarray:
    """在图像上绘制最小外接矩形 + 较长边方向箭头。

    - 绿色多边形: 最小外接矩形 4 角点
    - 红色箭头: 从矩形中心沿较长边方向，长度 = 较长边全长 (双向)
    - 角度文本
    """
    vis = image.copy()
    box = np.int0(fit["box"])
    cv2.drawContours(vis, [box], 0, color_rect, 2)

    center = fit["center"]
    axis = fit["axis"]
    half = fit["length"] / 2.0

    p1 = center - axis * half  # 尾端
    p2 = center + axis * half  # 头端

    # 贯穿直线 (双向)
    cv2.line(
        vis,
        (int(p1[0]), int(p1[1])),
        (int(p2[0]), int(p2[1])),
        color_axis, 2,
    )
    # 头端箭头标识方向
    cv2.arrowedLine(
        vis,
        (int(center[0]), int(center[1])),
        (int(p2[0]), int(p2[1])),
        color_axis, 2, tipLength=0.2,
    )

    cv2.circle(vis, (int(center[0]), int(center[1])), 4, (0, 255, 255), -1)

    label = f"ang={fit['angle_deg']:.1f}deg L={fit['length']:.0f}px"
    cv2.putText(
        vis, label,
        (int(center[0]) + 8, int(center[1]) - 8),
        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA,
    )
    return vis


class DinoPoseTestNode(Node):
    """ROS2 节点：订阅图像 → GroundingDINO 检测 bottle → 矩形拟合方向 → 显示。"""

    def __init__(self):
        super().__init__("dino_pose_test")

        # ── 参数 ──────────────────────────────────────────────
        self.declare_parameter("prompt", "bottle.")
        self.declare_parameter("image_topic", "/camera/camera_top/color/image_raw")
        self.declare_parameter("box_threshold", 0.35)
        self.declare_parameter("text_threshold", 0.25)
        self.declare_parameter("skip_frames", 0)  # 每隔 N 帧检测一次, 0=每帧

        self._prompt = self.get_parameter("prompt").value
        self._image_topic = self.get_parameter("image_topic").value
        self._skip_frames = self.get_parameter("skip_frames").value

        # ── 加载模型 (阻塞, 只执行一次) ────────────────────────
        self.get_logger().info("Loading Grounding DINO + SAM2 models …")
        self._wrapper = DinoWrapper(
            box_threshold=self.get_parameter("box_threshold").value,
            text_threshold=self.get_parameter("text_threshold").value,
        )
        self.get_logger().info("Models loaded successfully.")

        # ── 订阅图像话题 ──────────────────────────────────────
        self.create_subscription(
            Image, self._image_topic, self._image_callback, 10
        )

        self._frame_count = 0
        self._last_result = None  # 缓存最近一次检测结果用于跳帧显示

        self.get_logger().info(
            f'Subscribed to "{self._image_topic}", prompt="{self._prompt}". '
            "Press 'q' in the OpenCV window to exit."
        )

    def _image_callback(self, msg: Image):
        self._frame_count += 1

        # ── ROS Image → cv::Mat (BGR) ────────────────────────
        try:
            frame = ros_image_to_cv2(msg)
        except Exception as e:
            self.get_logger().error(f"Image conversion failed: {e}")
            return

        # ── 检查是否需要更新 prompt 参数 ──────────────────────
        new_prompt = self.get_parameter("prompt").value
        if new_prompt != self._prompt:
            self._prompt = new_prompt
            self.get_logger().info(f"Prompt updated to: {self._prompt!r}")

        # ── 跳帧逻辑 ─────────────────────────────────────────
        do_detect = (
            self._skip_frames == 0
            or self._frame_count % (self._skip_frames + 1) == 1
        )

        if do_detect:
            try:
                result = self._wrapper.detect(frame, self._prompt)
                self._last_result = result
            except Exception as e:
                self.get_logger().error(f"Detection failed: {e}")
                result = None
        else:
            result = self._last_result

        # ── 矩形拟合方向 + 可视化 ─────────────────────────────
        display = frame.copy()
        if result is not None and len(result.boxes) > 0:
            for i in range(len(result.boxes)):
                mask = result.masks[i]
                fit = fit_rectangle_orientation(mask)
                if fit is None:
                    continue
                display = draw_orientation(display, fit)
                self.get_logger().info(
                    f"obj{i}: angle={fit['angle_deg']:.1f}deg "
                    f"len={fit['length']:.1f}px",
                    throttle_duration_sec=1.0,
                )
            cv2.putText(
                display,
                f"dets={len(result.boxes)}",
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                (0, 255, 255), 2,
            )
        elif not do_detect and self._last_result is None:
            cv2.putText(
                display, "waiting for detection ...",
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                (128, 128, 128), 1,
            )

        cv2.imshow("Dino Pose Test", display)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            self.get_logger().info("User pressed 'q', shutting down …")
            rclpy.shutdown()


def main():
    rclpy.init(args=sys.argv)

    node = DinoPoseTestNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
