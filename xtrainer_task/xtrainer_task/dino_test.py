#!/usr/bin/env python3
"""Grounding DINO + SAM2 实时检测测试脚本。

订阅 ``/camera/camera_top/color/image_raw`` 话题，
调用 DinoWrapper 进行检测与分割，并通过 OpenCV imshow 显示结果。

.. note::

    此脚本不依赖 ``cv_bridge`` (后者与 conda numpy 2.x 冲突)，
    直接解析 ``sensor_msgs/Image`` 原始数据，支持 ``bgr8`` 与 ``rgb8`` 编码。

Usage::

    ros2 run xtrainer_task dino_test --ros-args -p prompt:="bottle. cup." -p image_topic:="/camera/camera_top/color/image_raw"

默认 prompt 为 "objects."，可通过 ROS 参数动态修改。
"""

import sys

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from xtrainer_task.dino_wrapper import DinoWrapper

_SUPPORTED_ENCODINGS = {"bgr8", "rgb8"}


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


class DinoTestNode(Node):
    """ROS2 节点：订阅图像话题 → DinoWrapper 检测 → OpenCV 显示。"""

    def __init__(self):
        super().__init__("dino_test")

        # ── 参数 ──────────────────────────────────────────────
        self.declare_parameter("prompt", "objects.")
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
        self._sub = self.create_subscription(
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
        if self._skip_frames > 0 and self._frame_count % (self._skip_frames + 1) != 1:
            # 非检测帧, 用缓存结果显示
            if self._last_result is not None:
                display = DinoWrapper.annotate(frame, self._last_result)
            else:
                display = frame
        else:
            # 执行检测
            try:
                result = self._wrapper.detect(frame, self._prompt)
                self._last_result = result
                display = DinoWrapper.annotate(frame, result)
                self.get_logger().info(
                    f"Frame #{self._frame_count}: {len(result.boxes)} detections",
                    throttle_duration_sec=1.0,
                )
            except Exception as e:
                self.get_logger().error(f"Detection failed: {e}")
                display = frame

        # ── OpenCV 显示 ──────────────────────────────────────
        cv2.imshow("Dino Test", display)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            self.get_logger().info("User pressed 'q', shutting down …")
            rclpy.shutdown()


def main():
    rclpy.init(args=sys.argv)

    node = DinoTestNode()

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