#!/usr/bin/env python3
"""SAM2AutomaticMaskGenerator 实时自动分割测试脚本。

订阅 ``/camera/camera_top/color/image_raw`` 彩色图，使用 SAM2
**自动掩码生成器** (无需文本 prompt) 检测并分割画面中的所有物体，
通过 OpenCV ``imshow`` 显示彩色掩码叠加 + 轮廓 + bbox + 编号。

与 ``detect_desk_object.py`` / ``dino_test.py`` 的区别:
    - 不需要 Grounding DINO, 不需要 ROI, 不需要点云/GraspNet。
    - 纯 SAM2 全图自动分割, 对画面中一切物体生成掩码。

SAM2AutomaticMaskGenerator 参数 (可运行时通过 ROS 参数调整):

    =========================== =========== ====================================
    参数                        默认值      作用 / 调整建议
    =========================== =========== ====================================
    points_per_side             32          网格采样密度; 小物体/密集物体可调
                                            到 48~64 (代价: 显存与耗时上升)
    pred_iou_thresh             0.85        预测质量过滤; 边界模糊丢候选时可降到
                                            0.80
    stability_score_thresh      0.90        稳定性过滤; 可降到 0.85
    min_mask_region_area        0           最小区域(像素); 按物体大小设置过滤噪
                                            声碎片
    crop_n_layers               0           多尺度裁剪层数; 调到 1 对小物体/边缘
                                            物体召回帮助大, 但每个裁剪块都重新
                                            跑编码器, 显存与耗时显著上升
    box_nms_thresh              0.7         重叠框去重; 物体紧邻时调高到 0.8~0.9
                                            避免误删相邻正确 mask
    =========================== =========== ====================================

Usage::

    export PYTHONPATH=/opt/Project/Grounded-SAM-2/grounding_dino:/opt/Project/Grounded-SAM-2
    ros2 run xtrainer_task sam_detection_test --ros-args \
        -p points_per_side:=64 \
        -p pred_iou_thresh:=0.80 \
        -p stability_score_thresh:=0.85 \
        -p min_mask_region_area:=500 \
        -p box_nms_thresh:=0.9

按键: OpenCV 窗口按 ``q`` 退出。

.. note::

    不依赖 ``cv_bridge`` (与 conda numpy 2.x 冲突), 直接解析
    ``sensor_msgs/Image`` 原始数据 (同 ``dino_test.py``)。
    默认配置 (base_plus + points_per_side=32 + crop_n_layers=0) 约 <1s/帧;
    推断在后台线程执行, 始终处理最新帧, 不阻塞 ROS 回调。
"""

import sys
import threading
import time

import cv2
import numpy as np
import rclpy
import torch
from rclpy.node import Node
from sam2.automatic_mask_generator import SAM2AutomaticMaskGenerator
from sam2.build_sam import build_sam2
from sensor_msgs.msg import Image

from xtrainer_task.dino_test import ros_image_to_cv2

# ── 绘制用高对比调色板 (BGR) ─────────────────────────────────
_PALETTE = [
    (255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 0),
    (0, 255, 255), (255, 0, 255), (255, 128, 0), (128, 0, 255),
    (0, 128, 255), (128, 255, 0), (255, 128, 128), (128, 255, 128),
    (255, 0, 128), (0, 128, 128), (180, 105, 255), (64, 64, 255),
]


def _color(i: int):
    return _PALETTE[i % len(_PALETTE)]


class SamDetectionTestNode(Node):
    """ROS2 节点: 订阅图像 → SAM2 自动掩码 → OpenCV 显示。"""

    def __init__(self):
        super().__init__("sam_detection_test")

        # ── 参数 ──────────────────────────────────────────────
        self.declare_parameter(
            "image_topic", "/camera/camera_top/color/image_raw"
        )
        self.declare_parameter(
            "sam2_checkpoint",
            "/opt/Project/Grounded-SAM-2/checkpoints/sam2.1_hiera_base_plus.pt",
        )
        self.declare_parameter(
            "sam2_config", "configs/sam2.1/sam2.1_hiera_b+.yaml"
        )
        # SAM2AutomaticMaskGenerator 参数 (默认值取调优建议区间)
        self.declare_parameter("points_per_side", 32)
        self.declare_parameter("pred_iou_thresh", 0.85)
        self.declare_parameter("stability_score_thresh", 0.90)
        self.declare_parameter("min_mask_region_area", 0)
        self.declare_parameter("crop_n_layers", 0)
        self.declare_parameter("box_nms_thresh", 0.7)
        self.declare_parameter("min_infer_interval", 0.0)

        self._image_topic = self.get_parameter("image_topic").value
        self._points_per_side = self.get_parameter("points_per_side").value
        self._pred_iou_thresh = self.get_parameter("pred_iou_thresh").value
        self._stability_score_thresh = (
            self.get_parameter("stability_score_thresh").value
        )
        self._min_mask_region_area = (
            self.get_parameter("min_mask_region_area").value
        )
        self._crop_n_layers = self.get_parameter("crop_n_layers").value
        self._box_nms_thresh = self.get_parameter("box_nms_thresh").value
        self._min_infer_interval = self.get_parameter("min_infer_interval").value

        # ── 加载 SAM2 (不需要 Grounding DINO) ─────────────────
        device = "cuda" if torch.cuda.is_available() else "cpu"
        self.get_logger().info(
            f"Loading SAM2 model (device={device}) …"
        )
        self._sam2_model = build_sam2(
            self.get_parameter("sam2_config").value,
            self.get_parameter("sam2_checkpoint").value,
            device=device,
        )

        # ── 构建自动掩码生成器 ────────────────────────────────
        self._amg = SAM2AutomaticMaskGenerator(
            model=self._sam2_model,
            points_per_side=self._points_per_side,
            pred_iou_thresh=self._pred_iou_thresh,
            stability_score_thresh=self._stability_score_thresh,
            min_mask_region_area=self._min_mask_region_area,
            crop_n_layers=self._crop_n_layers,
            box_nms_thresh=self._box_nms_thresh,
            output_mode="binary_mask",
        )
        self.get_logger().info(
            f"SAM2 ready: points_per_side={self._points_per_side}, "
            f"pred_iou_thresh={self._pred_iou_thresh}, "
            f"stability_score_thresh={self._stability_score_thresh}, "
            f"crop_n_layers={self._crop_n_layers}, "
            f"box_nms_thresh={self._box_nms_thresh}."
        )

        # ── 流式状态 ──────────────────────────────────────────
        self._latest_image = None
        self._buf_lock = threading.Lock()
        self._running = True
        self._last_log = 0.0

        # ── 订阅 ───────────────────────────────────────────────
        self.create_subscription(
            Image, self._image_topic, self._image_callback, 10
        )

        self.get_logger().info(
            f'Subscribed to "{self._image_topic}". '
            "Press 'q' in the OpenCV window to exit."
        )

        # ── 启动推理线程 ──────────────────────────────────────
        self._worker = threading.Thread(
            target=self._inference_loop, daemon=True
        )
        self._worker.start()

    # ── 回调: 只缓存最新帧 ─────────────────────────────────────
    def _image_callback(self, msg: Image):
        with self._buf_lock:
            self._latest_image = msg

    # ── 后台推理线程 ──────────────────────────────────────────
    def _inference_loop(self):
        while self._running and rclpy.ok():
            with self._buf_lock:
                image_msg = self._latest_image
                if image_msg is not None:
                    self._latest_image = None

            if image_msg is None:
                time.sleep(0.02)
                continue

            t0 = time.time()
            try:
                self._process_frame(image_msg)
            except Exception as e:
                self.get_logger().error(f"Frame processing failed: {e}")

            dt = time.time() - t0
            if self._min_infer_interval > dt:
                time.sleep(self._min_infer_interval - dt)

    # ── 单帧处理 ──────────────────────────────────────────────
    def _process_frame(self, image_msg: Image):
        t0 = time.time()

        # ── ROS Image → BGR ──────────────────────────────────
        frame = ros_image_to_cv2(image_msg)

        # ── 热更新参数 (允许运行时调参不重启) ─────────────────
        self._maybe_refresh_amg()

        # ── SAM2 自动掩码生成 ────────────────────────────────
        anns = self._amg.generate(frame)

        # ── 过滤过小区域 ─────────────────────────────────────
        if self._min_mask_region_area > 0:
            anns = [a for a in anns if a["area"] >= self._min_mask_region_area]

        # ── 按面积降序 (大物体在前) ──────────────────────────
        anns.sort(key=lambda a: a["area"], reverse=True)

        # ── 绘制 ─────────────────────────────────────────────
        display = self._draw(frame, anns)
        n = len(anns)
        cv2.putText(
            display,
            f"masks={n} ({time.time() - t0:.2f}s)",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2,
        )
        cv2.imshow("SAM2 Auto Mask", display)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            self.get_logger().info("Pressed 'q', exiting …")
            self._running = False

        # ── 日志 ─────────────────────────────────────────────
        now = time.time()
        if now - self._last_log > 1.0:
            self._last_log = now
            self.get_logger().info(f"masks={n} ({now - t0:.2f}s)")
            self._log_masks(anns)

    # ── 运行时调参: 若关键参数变化则重建生成器 ─────────────────
    def _maybe_refresh_amg(self):
        pps = self.get_parameter("points_per_side").value
        pit = self.get_parameter("pred_iou_thresh").value
        sst = self.get_parameter("stability_score_thresh").value
        mmr = self.get_parameter("min_mask_region_area").value
        cnl = self.get_parameter("crop_n_layers").value
        bnt = self.get_parameter("box_nms_thresh").value

        if (
            pps == self._points_per_side
            and pit == self._pred_iou_thresh
            and sst == self._stability_score_thresh
            and mmr == self._min_mask_region_area
            and cnl == self._crop_n_layers
            and bnt == self._box_nms_thresh
        ):
            return

        self._points_per_side = pps
        self._pred_iou_thresh = pit
        self._stability_score_thresh = sst
        self._min_mask_region_area = mmr
        self._crop_n_layers = cnl
        self._box_nms_thresh = bnt

        self._amg = SAM2AutomaticMaskGenerator(
            model=self._sam2_model,
            points_per_side=self._points_per_side,
            pred_iou_thresh=self._pred_iou_thresh,
            stability_score_thresh=self._stability_score_thresh,
            min_mask_region_area=0,  # 面积过滤放到 _process_frame 统一处理
            crop_n_layers=self._crop_n_layers,
            box_nms_thresh=self._box_nms_thresh,
            output_mode="binary_mask",
        )
        self.get_logger().info(
            "Rebuilt SAM2AutomaticMaskGenerator: "
            f"points_per_side={pps}, pred_iou_thresh={pit}, "
            f"stability_score_thresh={sst}, min_mask_region_area={mmr}, "
            f"crop_n_layers={cnl}, box_nms_thresh={bnt}."
        )

    # ── OpenCV 可视化 ────────────────────────────────────────
    def _draw(self, frame, anns):
        display = frame.copy()
        if not anns:
            return display

        # ── 半透明彩色掩码叠加 ───────────────────────────────
        overlay = display.copy()
        for i, a in enumerate(anns):
            overlay[a["segmentation"]] = _color(i)
        display = cv2.addWeighted(display, 0.5, overlay, 0.5, 0)

        # ── 轮廓 + bbox + 编号 ───────────────────────────────
        for i, a in enumerate(anns):
            color = _color(i)
            mask_u8 = a["segmentation"].astype(np.uint8)
            cnts, _ = cv2.findContours(
                mask_u8, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )
            cv2.drawContours(display, cnts, -1, color, 2)

            x, y, w, h = a["bbox"]
            x, y, w, h = int(x), int(y), int(w), int(h)
            cv2.rectangle(display, (x, y), (x + w, y + h), color, 1)
            cx, cy = x + w // 2, y + h // 2
            cv2.putText(
                display, f"{i}", (cx, cy),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2,
            )

        return display

    # ── 日志输出掩码信息 ─────────────────────────────────────
    def _log_masks(self, anns):
        if not anns:
            return
        self.get_logger().info(f"—— SAM2 automatic masks: {len(anns)} ——")
        for i, a in enumerate(anns):
            x, y, w, h = a["bbox"]
            self.get_logger().info(
                f"#{i}: bbox=({int(x)},{int(y)})-({int(x + w)},{int(y + h)}) "
                f"area={a['area']} iou={a['predicted_iou']:.3f} "
                f"stab={a['stability_score']:.3f}"
            )


def main():
    rclpy.init(args=sys.argv)

    node = SamDetectionTestNode()

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)

    try:
        while rclpy.ok() and node._running:
            executor.spin_once(timeout_sec=0.01)
    except KeyboardInterrupt:
        pass
    finally:
        node._running = False
        if node._worker.is_alive():
            node._worker.join(timeout=2.0)
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
