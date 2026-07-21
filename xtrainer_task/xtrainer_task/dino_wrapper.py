"""
DinoWrapper — Grounding DINO + SAM2 封装

在 __init__ 时加载 Grounding DINO 与 SAM2 模型，
提供 detect() 方法：输入 cv::Mat 图像与文本 prompt，输出检测结果。

Boot：
export PYTHONPATH=/opt/Project/Grounded-SAM-2/grounding_dino:/opt/Project/Grounded-SAM-2:/home/kira/miniconda3/lib/python3.12/site-packages:$PYTHONPATH
ros2 run xtrainer_task dino_test --ros-args -p prompt:="bottle. cup."

"""

from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Optional, Tuple

import cv2
import numpy as np
import supervision as sv
import torch
from groundingdino.util.inference import load_model, predict
from sam2.build_sam import build_sam2
from sam2.sam2_image_predictor import SAM2ImagePredictor
from torchvision.ops import box_convert

# ── 默认模型路径 (相对于 Grounded-SAM-2 仓库根目录) ──────────────────
_GSAM2_ROOT = Path("/opt/Project/Grounded-SAM-2")

_DEFAULT_SAM2_CHECKPOINT = str(_GSAM2_ROOT / "checkpoints" / "sam2.1_hiera_large.pt")
# SAM2 的 Hydra config 路径是相对于 sam2 包内部的 configs/ 目录
_DEFAULT_SAM2_CONFIG = "configs/sam2.1/sam2.1_hiera_l.yaml"
_DEFAULT_GDINO_CONFIG = str(_GSAM2_ROOT / "grounding_dino" / "groundingdino" / "config" / "GroundingDINO_SwinT_OGC.py")
_DEFAULT_GDINO_CHECKPOINT = str(_GSAM2_ROOT / "gdino_checkpoints" / "groundingdino_swint_ogc.pth")

# ImageNet 归一化参数 (与 load_image 一致)
_IMAGENET_MEAN = np.array([0.485, 0.456, 0.406], dtype=np.float32)
_IMAGENET_STD = np.array([0.229, 0.224, 0.225], dtype=np.float32)


@dataclass
class DetectionResult:
    """单次检测结果"""
    boxes: np.ndarray           # (N, 4) xyxy 格式, 像素坐标
    masks: np.ndarray           # (N, H, W) bool 数组
    scores: List[float]         # 置信度列表
    labels: List[str]           # 类别标签列表
    image_height: int
    image_width: int

    def __post_init__(self):
        if self.masks.ndim == 4:
            self.masks = self.masks.squeeze(1)


class DinoWrapper:
    """Grounding DINO 检测 + SAM2 分割 封装类"""

    def __init__(
        self,
        sam2_checkpoint: str = _DEFAULT_SAM2_CHECKPOINT,
        sam2_config: str = _DEFAULT_SAM2_CONFIG,
        gdino_config: str = _DEFAULT_GDINO_CONFIG,
        gdino_checkpoint: str = _DEFAULT_GDINO_CHECKPOINT,
        box_threshold: float = 0.35,
        text_threshold: float = 0.25,
        multimask_output: bool = False,
        device: Optional[str] = None,
    ):
        self.box_threshold = box_threshold
        self.text_threshold = text_threshold
        self.multimask_output = multimask_output
        self.device = device or ("cuda" if torch.cuda.is_available() else "cpu")

        # ── 加载 SAM2 ────────────────────────────────────────
        self.sam2_model = build_sam2(sam2_config, sam2_checkpoint, device=self.device)
        self.sam2_predictor = SAM2ImagePredictor(self.sam2_model)

        # ── 加载 Grounding DINO ──────────────────────────────
        self.grounding_model = load_model(
            model_config_path=gdino_config,
            model_checkpoint_path=gdino_checkpoint,
            device=self.device,
        )

        # ── 性能优化: Ampere+ GPU 开启 tf32 ──────────────────
        if torch.cuda.is_available() and torch.cuda.get_device_properties(0).major >= 8:
            torch.backends.cuda.matmul.allow_tf32 = True
            torch.backends.cudnn.allow_tf32 = True

    @staticmethod
    def _bgr_to_gdino_tensor(image_bgr: np.ndarray) -> torch.Tensor:
        """将 BGR numpy 图像转换为 Grounding DINO 期望的归一化 tensor。

        等效于 ``groundingdino.util.inference.load_image()`` 的 transform：
        ``RandomResize([800], max_size=1333)`` → ``ToTensor`` → ``Normalize``。

        为避免 torchvision 版本不兼容 (v1/v2 API 差异)，手动实现全部步骤。
        """
        h, w = image_bgr.shape[:2]

        # ── 1. 仿 RandomResize([800], max_size=1333) ────────
        #     最短边缩放到 800，保持宽高比，最长边不超过 1333
        short_side = 800
        max_size = 1333
        scale = short_side / min(h, w)
        if max(h, w) * scale > max_size:
            scale = max_size / max(h, w)
        new_w, new_h = int(round(w * scale)), int(round(h * scale))

        # BGR → RGB → resize
        image_rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
        image_rgb = cv2.resize(image_rgb, (new_w, new_h), interpolation=cv2.INTER_LINEAR)

        # ── 2. 仿 ToTensor: HWC→CHW, uint8→float32 [0, 1] ──
        tensor = torch.from_numpy(image_rgb).permute(2, 0, 1).float() / 255.0

        # ── 3. 仿 Normalize([0.485,0.456,0.406],[0.229,0.224,0.225]) ─
        mean = torch.tensor(_IMAGENET_MEAN, device=tensor.device).view(3, 1, 1)
        std = torch.tensor(_IMAGENET_STD, device=tensor.device).view(3, 1, 1)
        tensor = (tensor - mean) / std

        return tensor  # (3, H, W), float32, normalized

    def detect(self, image: np.ndarray, prompt: str) -> DetectionResult:
        """
        对输入图像执行 Grounding DINO 检测 + SAM2 分割。

        Parameters
        ----------
        image : np.ndarray
            cv::Mat 格式图像 (BGR, HWC, uint8)
        prompt : str
            文本提示, 小写并以 '.' 结尾, 如 "car. tire."

        Returns
        -------
        DetectionResult
            包含 boxes, masks, scores, labels 的结果对象
        """
        h, w = image.shape[:2]

        # ── 1. SAM2 set_image ───────────────────────────────
        self.sam2_predictor.set_image(image)

        # ── 2. Grounding DINO 检测 ──────────────────────────
        image_tensor = self._bgr_to_gdino_tensor(image)
        boxes_ccwh, confidences, labels = predict(
            model=self.grounding_model,
            image=image_tensor,
            caption=prompt,
            box_threshold=self.box_threshold,
            text_threshold=self.text_threshold,
            device=self.device,
        )

        if len(boxes_ccwh) == 0:
            return DetectionResult(
                boxes=np.empty((0, 4)),
                masks=np.empty((0, h, w), dtype=bool),
                scores=[],
                labels=[],
                image_height=h,
                image_width=w,
            )

        # boxes_ccwh 是归一化坐标 (0~1)，转为像素 xyxy
        boxes_ccwh = boxes_ccwh * torch.Tensor([w, h, w, h])
        input_boxes = box_convert(boxes=boxes_ccwh, in_fmt="cxcywh", out_fmt="xyxy").numpy()

        # ── 3. SAM2 分割 (bfloat16 autocast) ────────────────
        with torch.autocast(device_type=self.device, dtype=torch.bfloat16):
            masks, scores, _ = self.sam2_predictor.predict(
                point_coords=None,
                point_labels=None,
                box=input_boxes,
                multimask_output=self.multimask_output,
            )

        # ── 4. 后处理 ───────────────────────────────────────
        if self.multimask_output:
            best = np.argmax(scores, axis=1)
            masks = masks[np.arange(masks.shape[0]), best]

        if masks.ndim == 4:
            masks = masks.squeeze(1)

        return DetectionResult(
            boxes=input_boxes,
            masks=masks.astype(bool),
            scores=scores.tolist() if isinstance(scores, np.ndarray) else list(scores),
            labels=list(labels),
            image_height=h,
            image_width=w,
        )

    @staticmethod
    def annotate(
        image: np.ndarray,
        result: DetectionResult,
        draw_mask: bool = True,
    ) -> np.ndarray:
        """可视化检测结果, 在原图上绘制框 + 标签 + (可选) mask。"""
        if len(result.boxes) == 0:
            return image.copy()

        class_ids = np.arange(len(result.labels))
        label_texts = [
            f"{name} {score:.2f}"
            for name, score in zip(result.labels, result.scores)
        ]

        detections = sv.Detections(
            xyxy=result.boxes,
            mask=result.masks,
            class_id=class_ids,
        )

        frame = image.copy()
        box_annotator = sv.BoxAnnotator()
        frame = box_annotator.annotate(scene=frame, detections=detections)

        label_annotator = sv.LabelAnnotator()
        frame = label_annotator.annotate(scene=frame, detections=detections, labels=label_texts)

        if draw_mask:
            mask_annotator = sv.MaskAnnotator()
            frame = mask_annotator.annotate(scene=frame, detections=detections)

        return frame