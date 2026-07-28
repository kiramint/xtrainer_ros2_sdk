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

        # Circle Test
        if len(result.boxes) != 0:
            bbox = [result.boxes[0, 0],result.boxes[0, 1],result.boxes[0, 2],result.boxes[0, 3]]
            circle_result = get_cap_center_hough(frame,bbox)

            mid_point = circle_result["center"]
            annotated_image = circle_result["vis_img"]

            if mid_point is not None and annotated_image is not None:
                cv2.imshow("Detection Result", annotated_image)

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