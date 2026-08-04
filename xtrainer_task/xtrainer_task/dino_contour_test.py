#!/usr/bin/env python3
"""Grounding DINO 识别 white square + OpenCV 轮廓检测测试脚本 (无 SAM)。

流程:
    1. 订阅 ``/camera/camera_top/color/image_raw`` (彩色图)。
    2. 仅用 Grounding DINO 检测 ``prompt`` (默认 "white square.")，得到
       detection boxes (不含 SAM mask)。
    3. 在每个 detection box 区域内做灰度阈值 → ``findContours``，
       显示 **全部** 轮廓。
    4. OpenCV 窗口显示结果。

显示:
    - 窗口 "Dino + Contour": detection box + 所有轮廓 + 面积

按键:
    按 ``q`` 退出。

Boot::

    export PYTHONPATH=/opt/Project/Grounded-SAM-2/grounding_dino:/home/kira/miniconda3/lib/python3.12/site-packages:$PYTHONPATH
    ros2 run xtrainer_task dino_contour_test --ros-args -p prompt:="white square."

.. note::

    不依赖 ``cv_bridge``, 直接解析 ROS 原始消息 (同 ``dino_test.py``)。
"""

import sys

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from xtrainer_task.dino_test import ros_image_to_cv2
from xtrainer_task.dino_wrapper import DinoWrapper


class DinoContourTestNode(Node):
    """ROS2 节点: 仅 GroundingDINO 识别 → OpenCV 显示全部轮廓。"""

    def __init__(self):
        super().__init__("dino_contour_test")

        # ── 参数 ──────────────────────────────────────────────
        self.declare_parameter("prompt", "white square.")
        self.declare_parameter("image_topic", "/camera/camera_top/color/image_raw")
        self.declare_parameter("box_threshold", 0.35)
        self.declare_parameter("text_threshold", 0.25)
        self.declare_parameter("skip_frames", 0)  # 每隔 N 帧检测一次, 0=每帧
        # 灰度阈值 (0-255): 在 box 区域内按灰度提取轮廓
        self.declare_parameter("gray_threshold", 220)
        # CLAHE 对比度增强 (0-4, 0=关闭, 越大对比越强)
        self.declare_parameter("clahe_clip", 2.0)
        # 锐度增强强度 (0=关闭, 越大越锐利)
        self.declare_parameter("sharpen_amount", 1.0)
        # 轮廓面积下限 (相对图像面积比例, 0~1), 过滤噪点
        self.declare_parameter("min_area_ratio", 0.0005)
        # box 外扩像素: 让阈值轮廓能完整包含白色方块边缘
        self.declare_parameter("box_margin_px", 10)

        self._prompt = self.get_parameter("prompt").value
        self._image_topic = self.get_parameter("image_topic").value
        self._skip_frames = self.get_parameter("skip_frames").value
        self._gray_threshold = self.get_parameter("gray_threshold").value
        self._clahe_clip = self.get_parameter("clahe_clip").value
        self._sharpen_amount = self.get_parameter("sharpen_amount").value
        self._min_area_ratio = self.get_parameter("min_area_ratio").value
        self._box_margin = self.get_parameter("box_margin_px").value

        # ── 加载模型 (仅 GroundingDINO, 不加载 SAM2) ─────────
        self.get_logger().info("Loading Grounding DINO model (no SAM2) …")
        self._wrapper = DinoWrapper(
            box_threshold=self.get_parameter("box_threshold").value,
            text_threshold=self.get_parameter("text_threshold").value,
            use_sam=False,
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

    # ── 图像增强 (对比度 + 锐度) ─────────────────────────────
    def _enhance_gray(self, gray):
        """CLAHE 对比度增强 + 反锐化掩模锐化, 提升白色方块边缘检测率。"""
        img = gray

        # 1. CLAHE 对比度增强 (自适应直方图均衡, 增强局部对比)
        if self._clahe_clip > 0:
            clahe = cv2.createCLAHE(
                clipLimit=self._clahe_clip, tileGridSize=(8, 8)
            )
            img = clahe.apply(img)

        # 2. 反锐化掩模 (Unsharp Mask) 锐化
        if self._sharpen_amount > 0:
            blur = cv2.GaussianBlur(img, (0, 0), sigmaX=1.0)
            img = cv2.addWeighted(
                img, 1.0 + self._sharpen_amount,
                blur, -self._sharpen_amount,
                0,
            )

        return img

    # ── OpenCV 轮廓检测 (在单个 box 内) ─────────────────────
    def _find_contours_in_box(self, gray, box, H, W):
        """在 box 区域内做灰度阈值 → 返回全部轮廓 (全图坐标)。

        Returns
        -------
        list[np.ndarray]
            满足面积下限的所有轮廓, 可能为空列表。
        """
        min_area = int(self._min_area_ratio * H * W)
        m = self._box_margin

        x1 = max(0, int(box[0]) - m)
        y1 = max(0, int(box[1]) - m)
        x2 = min(W - 1, int(box[2]) + m)
        y2 = min(H - 1, int(box[3]) + m)
        if x2 <= x1 or y2 <= y1:
            return []

        roi = self._enhance_gray(gray[y1:y2 + 1, x1:x2 + 1])

        cv2.imshow("ROI", roi)  # 显示 box 区域灰度图 (增强后)
        cv2.waitKey(1)

        _, thresh = cv2.threshold(roi, self._gray_threshold, 255, cv2.THRESH_BINARY)
        cnts, _ = cv2.findContours(
            thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        # 坐标还原回全图, 过滤噪点
        out = []
        for c in cnts:
            c_full = c + np.array([[x1, y1]], dtype=np.int32)
            if cv2.contourArea(c_full) >= min_area:
                out.append(c_full)
        return out

    # ── 回调 ──────────────────────────────────────────────────
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
        run_detect = True
        if self._skip_frames > 0 and self._frame_count % (self._skip_frames + 1) != 1:
            run_detect = False

        display = frame.copy()
        n_dets = 0
        n_contours = 0

        if run_detect:
            try:
                result = self._wrapper.detect(frame, self._prompt)
                self._last_result = result
            except Exception as e:
                self.get_logger().error(f"Detection failed: {e}")
                result = self._last_result
        else:
            result = self._last_result

        if result is not None and len(result.boxes) > 0:
            n_dets = len(result.boxes)
            # detection box (蓝色)
            for box in result.boxes.astype(int):
                cv2.rectangle(
                    display,
                    (box[0], box[1]),
                    (box[2], box[3]),
                    (255, 0, 0), 2,
                )

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            H, W = gray.shape[:2]

            # 每个 box 内显示全部轮廓
            all_contours = []
            for box in result.boxes:
                all_contours.extend(
                    self._find_contours_in_box(gray, box, H, W)
                )
            n_contours = len(all_contours)

            cv2.drawContours(display, all_contours, -1, (0, 255, 0), 2)
            for i, c in enumerate(all_contours):
                # 每个轮廓面积标注在质心
                M = cv2.moments(c)
                if M["m00"] > 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    cv2.putText(
                        display, f"{int(cv2.contourArea(c))}",
                        (cx, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (0, 255, 255), 1,
                    )

        # ── 顶部信息 ─────────────────────────────────────────
        cv2.putText(
            display, f"dets={n_dets} contours={n_contours}", (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2,
        )

        # ── OpenCV 显示 ──────────────────────────────────────
        cv2.imshow("Dino + Contour", display)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            self.get_logger().info("User pressed 'q', shutting down …")
            rclpy.shutdown()


def main():
    rclpy.init(args=sys.argv)

    node = DinoContourTestNode()

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
