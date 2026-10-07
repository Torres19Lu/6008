"""Lightweight fixed-target perception for color + object queries.

This module deliberately replaces CLIP with deterministic HSV color scoring.
It reuses the normal locator's depth, TF, multi-frame fusion and output contract.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import re
import time
from typing import Optional, Protocol, Sequence, Tuple

import cv2
import numpy as np

try:
    from .contracts import Detection2D, FrameSource, SynchronizedFrame, TargetQuery
except ImportError:
    from contracts import Detection2D, FrameSource, SynchronizedFrame, TargetQuery


@dataclass(frozen=True)
class RawDetection:
    """Detector-neutral normalized xyxy result."""

    class_name: str
    box: Tuple[float, float, float, float]
    confidence: float


class ObjectDetector(Protocol):
    def detect(self, frame: SynchronizedFrame, base_class: str) -> Sequence[RawDetection]: ...


# Canonical keys are used only by the HSV scorer.  Longer aliases are checked first.
COLOR_ALIASES = {
    "red": ("红色", "红", "red"),
    "orange": ("橙色", "橙", "orange"),
    "yellow": ("黄色", "黄", "yellow"),
    "green": ("绿色", "绿", "green"),
    "cyan": ("青色", "青", "cyan"),
    "blue": ("蓝色", "蓝", "blue"),
    "purple": ("紫色", "紫", "purple", "violet"),
    "pink": ("粉红色", "粉色", "粉", "pink"),
    "brown": ("棕色", "褐色", "棕", "brown"),
    "black": ("黑色", "黑", "black"),
    "white": ("白色", "白", "white"),
    "gray": ("灰色", "灰", "gray", "grey"),
}


# Common COCO names.  For uncommon classes callers can always send JSON with an
# explicit base_class instead of relying on this small natural-language parser.
OBJECT_ALIASES = {
    "chair": ("椅子", "座椅", "chairs", "chair"),
    "bottle": ("水瓶", "瓶子", "bottles", "bottle"),
    "cup": ("马克杯", "水杯", "杯子", "mug", "cups", "cup"),
    "person": ("行人", "人员", "人", "people", "person"),
    "backpack": ("双肩包", "背包", "backpack", "bag"),
    "couch": ("沙发", "sofa", "couch"),
    "dining table": ("餐桌", "桌子", "table", "desk"),
    "potted plant": ("盆栽", "植物", "potted plant", "plant"),
    "tv": ("显示器", "电视机", "电视", "television", "monitor", "tv"),
    "laptop": ("笔记本电脑", "电脑", "notebook", "laptop"),
    "cell phone": ("手机", "电话", "cell phone", "phone"),
    "book": ("书本", "书", "books", "book"),
    "bed": ("床", "beds", "bed"),
    "toilet": ("马桶", "toilets", "toilet"),
    "car": ("汽车", "车辆", "车", "cars", "car"),
    "bicycle": ("自行车", "单车", "bike", "bicycle"),
}


def _has_alias(text: str, alias: str) -> bool:
    folded = text.casefold()
    alias_folded = alias.casefold()
    if any("\u4e00" <= character <= "\u9fff" for character in alias_folded):
        return alias_folded in folded
    return re.search(r"(?<![a-z0-9]){}(?![a-z0-9])".format(
        re.escape(alias_folded)
    ), folded) is not None


def extract_color(target_text: str) -> Optional[str]:
    matches = (
        (len(alias), canonical)
        for canonical, aliases in COLOR_ALIASES.items()
        for alias in aliases
        if _has_alias(target_text, alias)
    )
    return max(matches, default=(0, None))[1]


def parse_target_query(target_text: str, explicit_base_class: Optional[str] = None) -> TargetQuery:
    """Parse common Chinese/English fixed queries into the existing contract."""

    text = target_text.strip()
    if not text:
        raise ValueError("target text is empty")
    if explicit_base_class is not None and explicit_base_class.strip():
        return TargetQuery(text, explicit_base_class.strip().casefold())

    matches = (
        (len(alias), canonical)
        for canonical, aliases in OBJECT_ALIASES.items()
        for alias in aliases
        if _has_alias(text, alias)
    )
    _, base_class = max(matches, default=(0, None))
    if base_class is None:
        raise ValueError(
            "cannot infer the detector class from {!r}; send JSON with base_class".format(text)
        )
    return TargetQuery(text, base_class)


def color_score_bgr(image_bgr: np.ndarray, color: str) -> float:
    """Return the fraction of pixels matching one supported canonical color."""

    if image_bgr is None or image_bgr.size == 0:
        return 0.0
    hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
    hue, saturation, value = hsv[..., 0], hsv[..., 1], hsv[..., 2]

    if color == "red":
        mask = (((hue <= 10) | (hue >= 170)) & (saturation >= 80) & (value >= 45))
    elif color == "orange":
        mask = ((hue >= 10) & (hue <= 24) & (saturation >= 75) & (value >= 55))
    elif color == "yellow":
        mask = ((hue >= 24) & (hue <= 36) & (saturation >= 70) & (value >= 70))
    elif color == "green":
        mask = ((hue >= 36) & (hue <= 85) & (saturation >= 55) & (value >= 40))
    elif color == "cyan":
        mask = ((hue >= 85) & (hue <= 100) & (saturation >= 55) & (value >= 45))
    elif color == "blue":
        mask = ((hue >= 100) & (hue <= 130) & (saturation >= 65) & (value >= 40))
    elif color == "purple":
        mask = ((hue >= 130) & (hue <= 160) & (saturation >= 50) & (value >= 40))
    elif color == "pink":
        mask = (((hue >= 160) | (hue <= 5)) & (saturation >= 35) & (value >= 100))
    elif color == "brown":
        mask = ((hue >= 5) & (hue <= 25) & (saturation >= 50)
                & (value >= 35) & (value <= 190))
    elif color == "black":
        mask = value <= 60
    elif color == "white":
        mask = (saturation <= 40) & (value >= 180)
    elif color == "gray":
        mask = (saturation <= 55) & (value >= 60) & (value <= 210)
    else:
        raise ValueError("unsupported color: {}".format(color))
    return float(np.count_nonzero(mask)) / float(mask.size)


class LiteVisionLanguageBackend:
    """YOLO candidate detection plus optional deterministic color ranking."""

    def __init__(self, detector: ObjectDetector) -> None:
        self.detector = detector

    def detect(self, frame: SynchronizedFrame, query: TargetQuery) -> Sequence[Detection2D]:
        requested_color = extract_color(query.target_text)
        results = []
        for detection in self.detector.detect(frame, query.base_class):
            if detection.class_name.casefold() != query.base_class.casefold():
                continue
            if not math.isfinite(detection.confidence):
                continue
            x1, y1, x2, y2 = detection.box
            if not all(math.isfinite(value) for value in detection.box):
                continue
            if not 0.0 <= x1 < x2 <= 1.0 or not 0.0 <= y1 < y2 <= 1.0:
                continue
            px1 = max(0, min(frame.width - 1, int(x1 * frame.width)))
            py1 = max(0, min(frame.height - 1, int(y1 * frame.height)))
            px2 = max(px1 + 1, min(frame.width, int(x2 * frame.width)))
            py2 = max(py1 + 1, min(frame.height, int(y2 * frame.height)))
            if requested_color is None:
                semantic_score = 1.0
            else:
                semantic_score = color_score_bgr(frame.rgb[py1:py2, px1:px2], requested_color)
            results.append(Detection2D(
                class_name=query.base_class,
                box=detection.box,
                detector_confidence=max(0.0, min(1.0, detection.confidence)),
                text_similarity=semantic_score,
            ))
        return results


class RateLimitedFrameSource:
    """Drop old synchronized frames so inference runs at a bounded rate."""

    def __init__(self, source: FrameSource, rate_hz: float = 2.0) -> None:
        if rate_hz <= 0.0:
            raise ValueError("rate_hz must be positive")
        self.source = source
        self.min_interval_s = 1.0 / rate_hz
        self.last_stamp: Optional[float] = None

    def next_frame(self, timeout_s: float) -> Optional[SynchronizedFrame]:
        deadline = time.monotonic() + timeout_s
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                return None
            frame = self.source.next_frame(remaining)
            if frame is None:
                return None
            if self.last_stamp is None or frame.stamp - self.last_stamp >= self.min_interval_s:
                self.last_stamp = frame.stamp
                return frame
