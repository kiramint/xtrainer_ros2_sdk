#!/usr/bin/env python3
"""Grounding DINO + SAM → 凸包 ROI → 分水岭算法 物体切分流水线。

流程:
    1. GroundingDINO 识别 ``prompt`` (默认 "white square.") 并用 SAM 分割。
    2. 取 SAM 掩码凸包作为 ROI, 在 ROI 内用分水岭算法切分物体:
       - 背景标记 (sure background): SAM 掩码区域 = WhiteSquare 表面
       - 前景标记 (sure foreground): 非 SAM 掩码区域经距离变换后阈值提取
       - 在 ROI 灰度梯度图上运行分水岭
    3. 每个分水岭标签提取为独立物体, 标序号、轮廓和 bbox。

显示:
    - OpenCV 窗口 "Desk Object Detect 2":
        - 左上: GroundingDINO 检测 + SAM 掩码
        - 右上: 凸包边界 (红) + 前景标记 (彩色点)
        - 下排: 分水岭分割结果 (彩色填充 + 蓝色轮廓 + 黄色序号)

按键:
    按 ``q`` 退出。

Boot::

    export PYTHONPATH=/opt/Project/Grounded-SAM-2/grounding_dino:/opt/Project/Grounded-SAM-2
    ros2 run xtrainer_task detect_desk_object2 --ros-args \
        -p prompt:="white square." \
        -p image_topic:=/camera/camera_top/color/image_raw
"""

import sys

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from xtrainer_task.dino_test import ros_image_to_cv2
from xtrainer_task.dino_wrapper import DinoWrapper


class DetectDeskObject2Node(Node):
    """ROS2 节点: WhiteSquare 检测 → 凸包 ROI → 分水岭物体切分。"""

    def __init__(self):
        super().__init__("detect_desk_object2")

        self.declare_parameter("image_topic", "/camera/camera_top/color/image_raw")
        self.declare_parameter("prompt", "white square.")
        self.declare_parameter("box_threshold", 0.35)
        self.declare_parameter("text_threshold", 0.25)
        self.declare_parameter("skip_frames", 0)
        self.declare_parameter(
            "sam2_checkpoint",
            "/opt/Project/Grounded-SAM-2/checkpoints/sam2.1_hiera_base_plus.pt",
        )
        self.declare_parameter("sam2_config", "configs/sam2.1/sam2.1_hiera_b+.yaml")
        self.declare_parameter("bg_dilate_px", 3)
        self.declare_parameter("fg_dist_thresh", 0.3)
        self.declare_parameter("min_object_area", 200)
        self.declare_parameter("border_dilate_px", 5)

        self._image_topic = self.get_parameter("image_topic").value
        self._prompt = self.get_parameter("prompt").value
        self._skip_frames = self.get_parameter("skip_frames").value
        self._bg_dilate = self.get_parameter("bg_dilate_px").value
        self._fg_dist_thresh = self.get_parameter("fg_dist_thresh").value
        self._min_object_area = self.get_parameter("min_object_area").value
        self._border_dilate = self.get_parameter("border_dilate_px").value

        self.get_logger().info("Loading Grounding DINO + SAM2 models …")
        self._wrapper = DinoWrapper(
            box_threshold=self.get_parameter("box_threshold").value,
            text_threshold=self.get_parameter("text_threshold").value,
            sam2_checkpoint=self.get_parameter("sam2_checkpoint").value,
            sam2_config=self.get_parameter("sam2_config").value,
        )
        self.get_logger().info("Models loaded.")

        self._sub = self.create_subscription(
            Image, self._image_topic, self._image_callback, 10
        )
        self._frame_count = 0
        self._last_display = None

        self.get_logger().info(
            f'Subscribed to "{self._image_topic}", prompt="{self._prompt}". '
            "Press 'q' to exit."
        )

    def _watershed_segment(self, frame, sam_mask):
        """在 SAM 掩码凸包 ROI 内用 OpenCV 分水岭算法切分物体。

        Parameters
        ----------
        frame : np.ndarray
            BGR 原始图像。
        sam_mask : np.ndarray
            (H, W) bool SAM 掩码。

        Returns
        -------
        list[dict]
            每项含:
                - mask: (H, W) uint8 0/255 全图物体掩码
                - bbox: (x, y, w, h)
                - area: int
                - centroid: (cx, cy)
        np.ndarray or None
            (H, W) 可视化标记图 (彩色), 无物体时为 None。
        np.ndarray or None
            (H, W) 凸包边界掩码, 无轮廓时为 None。
        """
        H, W = frame.shape[:2]
        mask_u8 = sam_mask.astype(np.uint8) * 255
        cnts, _ = cv2.findContours(mask_u8, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not cnts:
            return [], None, None

        combined = np.vstack(cnts)
        hull = cv2.convexHull(combined)

        roi_mask = np.zeros((H, W), dtype=np.uint8)
        cv2.drawContours(roi_mask, [hull.astype(np.int32)], -1, 255, -1)

        if self._border_dilate > 0:
            kernel = cv2.getStructuringElement(
                cv2.MORPH_ELLIPSE,
                (self._border_dilate * 2 + 1, self._border_dilate * 2 + 1),
            )
            roi_mask = cv2.dilate(roi_mask, kernel)

        x, y, w, h_rect = cv2.boundingRect(hull.astype(np.int32))
        x, y, w, h_rect = [
            max(0, x), max(0, y),
            min(W - x, w), min(H - y, h_rect),
        ]
        if w <= 0 or h_rect <= 0:
            return [], None, roi_mask

        roi_frame = frame[y:y + h_rect, x:x + w]
        roi_sam_mask = roi_mask[y:y + h_rect, x:x + w]
        roi_sam = sam_mask[y:y + h_rect, x:x + w]

        gray = cv2.cvtColor(roi_frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)

        grad_x = cv2.Sobel(blurred, cv2.CV_64F, 1, 0, ksize=3)
        grad_y = cv2.Sobel(blurred, cv2.CV_64F, 0, 1, ksize=3)
        gradient = cv2.normalize(
            cv2.magnitude(grad_x, grad_y), None, 0, 255, cv2.NORM_MINMAX
        ).astype(np.uint8)

        # ① sure_bg (确定背景): 膨胀 SAM 掩码 = WhiteSquare 表面, label = 1
        kernel_bg = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        sure_bg = cv2.dilate(
            roi_sam.astype(np.uint8) * 255, kernel_bg, iterations=self._bg_dilate
        )

        # ② sure_fg (确定前景): 凸包内非 WhiteSquare 区域做距离变换, 取峰值
        fg_candidate = cv2.bitwise_and(roi_sam_mask, cv2.bitwise_not(sure_bg))
        dist = cv2.distanceTransform(fg_candidate, cv2.DIST_L2, cv2.DIST_MASK_PRECISE)
        max_dist = dist.max()
        if max_dist <= 0:
            return [], None, roi_mask

        _, sure_fg = cv2.threshold(
            dist, self._fg_dist_thresh * max_dist, 255, cv2.THRESH_BINARY
        )
        sure_fg = sure_fg.astype(np.uint8)

        # ③ markers: sure_fg→2,3,4... / sure_bg→1 / unknown→0
        num_labels, markers = cv2.connectedComponents(sure_fg)
        markers = markers + 1  # shift: 0→1, 1→2, ..., fg 从 2 开始

        # sure_bg 区域 → label 1
        markers[sure_bg > 0] = 1

        # unknown = roi - sure_bg - sure_fg (边界带), 标记为 0 让 watershed 计算
        unknown = cv2.subtract(cv2.subtract(roi_sam_mask, sure_bg), sure_fg)
        markers[unknown > 0] = 0

        # ④ 可视化前景种子 (调试用)
        markers_vis = np.zeros_like(roi_frame)
        for lbl in range(2, num_labels + 1):
            markers_vis[markers == lbl] = np.random.randint(0, 256, 3, dtype=np.uint8)

        # ⑤ 运行分水岭
        gradient_color = cv2.cvtColor(gradient, cv2.COLOR_GRAY2BGR)
        cv2.watershed(gradient_color, markers)

        # ⑥ 提取每个标签为独立物体
        objects = []
        for lbl in range(2, num_labels + 1):
            obj_mask_roi = np.zeros_like(roi_sam_mask, dtype=np.uint8)
            obj_mask_roi[markers == lbl] = 255

            kernel_close = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
            obj_mask_roi = cv2.morphologyEx(obj_mask_roi, cv2.MORPH_CLOSE, kernel_close)

            obj_mask_roi[roi_sam_mask == 0] = 0

            cnts_obj, _ = cv2.findContours(
                obj_mask_roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )
            if not cnts_obj:
                continue

            largest = max(cnts_obj, key=cv2.contourArea)
            area = cv2.contourArea(largest)
            if area < self._min_object_area:
                continue

            obj_clean = np.zeros_like(roi_sam_mask, dtype=np.uint8)
            cv2.drawContours(obj_clean, [largest], -1, 255, -1)

            bx, by, bw, bh = cv2.boundingRect(largest)

            full_mask = np.zeros((H, W), dtype=np.uint8)
            full_mask[y:y + h_rect, x:x + w] = obj_clean

            M = cv2.moments(largest)
            if M["m00"] > 0:
                cx = int(M["m10"] / M["m00"]) + x
                cy = int(M["m01"] / M["m00"]) + y
            else:
                cx, cy = bx + bw // 2 + x, by + bh // 2 + y

            objects.append({
                "mask": full_mask,
                "bbox": (bx + x, by + y, bw, bh),
                "area": int(area),
                "centroid": (cx, cy),
            })

        objects.sort(key=lambda o: o["area"], reverse=True)
        return objects, markers_vis, roi_mask

    def _draw_results(self, frame, det, sam_mask, objects, markers_vis, roi_mask):
        H, W = frame.shape[:2]

        annot = DinoWrapper.annotate(frame, det)

        marker_disp = frame.copy()
        if roi_mask is not None:
            roi_border = cv2.Canny(roi_mask, 100, 200)
            marker_disp[roi_border > 0] = (0, 0, 255)

            if markers_vis is not None:
                ys, xs = np.where(roi_mask > 0)
                if len(ys) > 0:
                    ry0, rx0 = ys.min(), xs.min()
                    rh, rw = markers_vis.shape[:2]
                    roi_crop_h = ys.max() - ry0 + 1
                    roi_crop_w = xs.max() - rx0 + 1
                    if rh == roi_crop_h and rw == roi_crop_w:
                        overlay = marker_disp[ry0:ry0 + rh, rx0:rx0 + rw]
                        mask_mv = (markers_vis > 0).any(axis=2)
                        overlay[mask_mv] = markers_vis[mask_mv]
                        marker_disp[ry0:ry0 + rh, rx0:rx0 + rw] = overlay

        result = frame.copy()
        colors = [
            (255, 0, 0), (0, 255, 0), (0, 0, 255),
            (255, 255, 0), (255, 0, 255), (0, 255, 255),
            (128, 0, 255), (255, 128, 0), (0, 128, 255), (128, 255, 128),
        ]
        for i, obj in enumerate(objects):
            color = colors[i % len(colors)]

            full_mask = obj["mask"]
            overlay = result.copy()
            overlay[full_mask > 0] = color
            result = cv2.addWeighted(result, 0.6, overlay, 0.4, 0)

            cnts_obj, _ = cv2.findContours(
                full_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )
            cv2.drawContours(result, cnts_obj, -1, color, 2)

            bx, by, bw, bh = obj["bbox"]
            cv2.rectangle(result, (bx, by), (bx + bw, by + bh), (0, 128, 255), 1)
            cx, cy = obj["centroid"]
            cv2.putText(
                result, f"{i}", (cx - 10, cy),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2,
            )

        row1 = np.hstack([annot, marker_disp])
        row2 = result

        out_h = H * 2
        out_w = max(row1.shape[1], row2.shape[1])
        canvas = np.full((out_h, out_w, 3), 255, dtype=np.uint8)
        canvas[:H, :row1.shape[1]] = row1
        canvas[H:, :row2.shape[1]] = row2

        cv2.putText(
            canvas,
            f"dets={len(det.boxes)} objects={len(objects)}",
            (10, H + 30),
            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2,
        )
        return canvas

    def _image_callback(self, msg: Image):
        self._frame_count += 1

        try:
            frame = ros_image_to_cv2(msg)
        except Exception as e:
            self.get_logger().error(f"Image conversion failed: {e}")
            return

        new_prompt = self.get_parameter("prompt").value
        if new_prompt != self._prompt:
            self._prompt = new_prompt
            self.get_logger().info(f"Prompt updated to: {self._prompt!r}")

        if self._skip_frames > 0 and self._frame_count % (self._skip_frames + 1) != 1:
            if self._last_display is not None:
                cv2.imshow("Desk Object Detect 2", self._last_display)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                rclpy.shutdown()
            return

        try:
            det = self._wrapper.detect(frame, self._prompt)

            if len(det.masks) > 0:
                sam_mask = np.any(det.masks, axis=0).astype(bool)
            else:
                sam_mask = np.zeros(frame.shape[:2], dtype=bool)

            objects, markers_vis, roi_mask = self._watershed_segment(frame, sam_mask)

            self._last_display = self._draw_results(
                frame, det, sam_mask, objects, markers_vis, roi_mask
            )
            cv2.imshow("Desk Object Detect 2", self._last_display)

            self.get_logger().info(
                f"Frame #{self._frame_count}: {len(det.boxes)} DINO dets, "
                f"{len(objects)} objects",
                throttle_duration_sec=1.0,
            )

        except Exception as e:
            self.get_logger().error(f"Processing failed: {e}")

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            self.get_logger().info("User pressed 'q', shutting down …")
            rclpy.shutdown()


def main():
    rclpy.init(args=sys.argv)
    node = DetectDeskObject2Node()

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
