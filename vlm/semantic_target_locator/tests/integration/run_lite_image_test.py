"""Repeatable real detector + lightweight HSV ranking test (no CLIP)."""

from __future__ import annotations

import json
from pathlib import Path
import sys

import cv2
import numpy as np
from PIL import Image


HERE = Path(__file__).resolve().parent
VLM_ROOT = HERE.parents[1]
WORKSPACE = VLM_ROOT.parents[2]
PLAN_A_ROOT = WORKSPACE / "vlfm"
sys.path.insert(0, str(VLM_ROOT))
sys.path.insert(0, str(PLAN_A_ROOT))

from semantic_target_locator.contracts import SynchronizedFrame  # noqa: E402
from semantic_target_locator.lite import (  # noqa: E402
    LiteVisionLanguageBackend, RawDetection, parse_target_query,
)
from vlfm.vlm.grounding_dino_hf import GroundingDINOHF  # noqa: E402


class GroundingDinoChairDetector:
    def __init__(self) -> None:
        import torch

        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.model = GroundingDINOHF(
            str(PLAN_A_ROOT / "data" / "plan_a" / "grounding-dino"),
            self.device,
            image_size=640,
        )

    def detect(self, frame: SynchronizedFrame, base_class: str):
        rgb = cv2.cvtColor(frame.rgb, cv2.COLOR_BGR2RGB)
        detections = self.model.predict(rgb, base_class + " .", 0.25, 0.2)
        return [
            RawDetection(
                class_name=base_class,
                box=tuple(float(value) for value in detections.boxes[index].tolist()),
                confidence=float(detections.logits[index]),
            )
            for index in range(detections.num_detections)
        ]


def main() -> None:
    image_path = HERE.parent / "assets" / "three_chairs_red_target.png"
    rgb = np.asarray(Image.open(image_path).convert("RGB"))
    bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    height, width = bgr.shape[:2]
    frame = SynchronizedFrame(
        stamp=1000.0,
        source_frame="camera_color_optical_frame",
        width=width,
        height=height,
        rgb=bgr,
    )
    query = parse_target_query("红色的椅子")
    detector = GroundingDinoChairDetector()
    backend = LiteVisionLanguageBackend(detector)
    candidates = list(backend.detect(frame, query))
    selected = max(candidates, key=lambda item: item.joint_confidence) if candidates else None

    overlay = bgr.copy()
    for candidate in candidates:
        x1, y1, x2, y2 = candidate.box
        p1 = (int(x1 * width), int(y1 * height))
        p2 = (int(x2 * width), int(y2 * height))
        color = (0, 255, 0) if candidate is selected else (0, 165, 255)
        cv2.rectangle(overlay, p1, p2, color, 3)
        cv2.putText(
            overlay,
            "det={:.3f} red={:.3f}".format(
                candidate.detector_confidence, candidate.text_similarity
            ),
            (p1[0], max(20, p1[1] - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            color,
            2,
        )

    output_dir = HERE / "test_outputs" / "lite_red_chair"
    output_dir.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(output_dir / "detections.png"), overlay)
    report = {
        "passed": selected is not None and selected.box == candidates[0].box,
        "input": {
            "target_text": query.target_text,
            "base_class": query.base_class,
            "image": str(image_path),
        },
        "model": {
            "detector": "IDEA-Research/grounding-dino-tiny (test adapter)",
            "ranker": "HSV red pixel fraction",
            "clip_loaded": False,
            "device": detector.device,
        },
        "candidates": [
            {
                "box_normalized_xyxy": [round(value, 6) for value in item.box],
                "detector_confidence": round(item.detector_confidence, 6),
                "color_score": round(item.text_similarity, 6),
                "joint_score": round(item.joint_confidence, 6),
                "selected": item is selected,
            }
            for item in candidates
        ],
    }
    (output_dir / "report.json").write_text(
        json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8"
    )
    print(json.dumps(report, indent=2, ensure_ascii=False))
    if not report["passed"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
