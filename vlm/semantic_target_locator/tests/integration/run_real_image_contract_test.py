"""Real image recognition + simulated ROS/SLAM-contract localization test.

The RGB inference is real (GroundingDINO + CLIP).  The aligned depth image,
CameraInfo and map<-camera transform are deterministic fixtures because no
robot or rosbag is available on this machine.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
from pathlib import Path
import sys
from typing import Optional, Sequence

import cv2
import numpy as np
from PIL import Image


HERE = Path(__file__).resolve().parent
VLM_ROOT = HERE.parents[1]
WORKSPACE = VLM_ROOT.parents[2]
PLAN_A_ROOT = WORKSPACE / "vlfm"
sys.path.insert(0, str(VLM_ROOT))
sys.path.insert(0, str(PLAN_A_ROOT))

from semantic_target_locator.contracts import (  # noqa: E402
    Detection2D,
    SynchronizedFrame,
    TargetQuery,
    Update,
)
from semantic_target_locator.geometry import MatrixTransformProvider  # noqa: E402
from semantic_target_locator.locator import LocatorConfig, SemanticTargetLocator  # noqa: E402
from semantic_target_locator.ros_adapter import AlignedDepthProvider  # noqa: E402
from vlfm.vlm.clip_itm import CLIPITM  # noqa: E402
from vlfm.vlm.grounding_dino_hf import GroundingDINOHF  # noqa: E402


@dataclass(frozen=True)
class CameraInfoFixture:
    K: tuple[float, ...]


class RepeatedFrameSource:
    def __init__(self, frame: SynchronizedFrame, count: int = 3) -> None:
        self._frame = frame
        self._remaining = count
        self._index = 0

    def next_frame(self, timeout_s: float) -> Optional[SynchronizedFrame]:
        del timeout_s
        if self._remaining <= 0:
            return None
        self._remaining -= 1
        self._index += 1
        return SynchronizedFrame(
            stamp=self._frame.stamp + 0.1 * self._index,
            source_frame=self._frame.source_frame,
            width=self._frame.width,
            height=self._frame.height,
            rgb=self._frame.rgb,
            depth=self._frame.depth,
            camera_info=self._frame.camera_info,
            metadata=self._frame.metadata,
        )


class CollectingSink:
    def __init__(self) -> None:
        self.updates: list[Update] = []

    def publish(self, update: Update) -> None:
        self.updates.append(update)


class PlanABackend:
    def __init__(self) -> None:
        import torch

        device = "cuda" if torch.cuda.is_available() else "cpu"
        self.device = device
        self.detector = GroundingDINOHF(
            str(PLAN_A_ROOT / "data" / "plan_a" / "grounding-dino"),
            device,
            image_size=640,
        )
        self.clip = CLIPITM(str(PLAN_A_ROOT / "data" / "plan_a" / "clip"), device)
        self.last_raw_count = 0
        self._cache_key = None
        self._cache: Sequence[Detection2D] = ()

    def detect(self, frame: SynchronizedFrame, query: TargetQuery) -> Sequence[Detection2D]:
        cache_key = (id(frame.rgb), query.target_text, query.base_class)
        if cache_key == self._cache_key:
            return self._cache
        detections = self.detector.predict(frame.rgb, query.base_class + " .", 0.25, 0.2)
        self.last_raw_count = detections.num_detections
        results = []
        for index in range(detections.num_detections):
            box = tuple(float(value) for value in detections.boxes[index].tolist())
            x1, y1, x2, y2 = box
            px1 = max(0, min(frame.width - 1, int(x1 * frame.width)))
            py1 = max(0, min(frame.height - 1, int(y1 * frame.height)))
            px2 = max(px1 + 1, min(frame.width, int(x2 * frame.width)))
            py2 = max(py1 + 1, min(frame.height, int(y2 * frame.height)))
            crop = frame.rgb[py1:py2, px1:px2]
            similarity = self.clip.cosine(crop, "a photo of a " + query.target_text)
            results.append(Detection2D(
                class_name=query.base_class,
                box=box,
                detector_confidence=float(detections.logits[index]),
                text_similarity=float(similarity),
            ))
        self._cache_key = cache_key
        self._cache = results
        return results


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--image",
        type=Path,
        default=PLAN_A_ROOT / "data" / "plan_a" / "sample.png",
    )
    parser.add_argument("--target-text", default="cat")
    parser.add_argument("--base-class", default="cat")
    parser.add_argument("--output-name", default="real_image_slam_contract")
    args = parser.parse_args()

    image_path = args.image.resolve()
    rgb = np.asarray(Image.open(image_path).convert("RGB"))
    height, width = rgb.shape[:2]

    # Same data contract as the ROS adapter: RGB + aligned uint16 depth in mm,
    # calibrated CameraInfo and a timestamped map<-camera optical transform.
    depth = np.full((height, width), 2000, dtype=np.uint16)
    focal = 0.82 * width
    camera_info = CameraInfoFixture((
        focal, 0.0, (width - 1) / 2.0,
        0.0, focal, (height - 1) / 2.0,
        0.0, 0.0, 1.0,
    ))
    sensor_to_map = (
        (0.0, 0.0, 1.0, 1.0),
        (-1.0, 0.0, 0.0, 2.0),
        (0.0, -1.0, 0.0, 1.0),
        (0.0, 0.0, 0.0, 1.0),
    )
    frame = SynchronizedFrame(
        stamp=1000.0,
        source_frame="camera_color_optical_frame",
        width=width,
        height=height,
        rgb=rgb,
        depth=depth,
        camera_info=camera_info,
        metadata={"depth_encoding": "16UC1", "sensor_to_map": sensor_to_map},
    )

    backend = PlanABackend()
    sink = CollectingSink()
    locator = SemanticTargetLocator(
        frames=RepeatedFrameSource(frame, count=3),
        vlm=backend,
        depth=AlignedDepthProvider(depth_scale_to_m=0.001),
        transforms=MatrixTransformProvider(),
        sink=sink,
        config=LocatorConfig(max_frames=3, min_observations=3),
    )
    query = TargetQuery(target_text=args.target_text, base_class=args.base_class)
    estimate = locator.locate(query)

    # Run once more to retain all actual boxes/scores for the human-readable report.
    candidates = list(backend.detect(frame, query))
    selected = max(candidates, key=lambda item: item.joint_confidence) if candidates else None
    overlay = cv2.cvtColor(rgb.copy(), cv2.COLOR_RGB2BGR)
    for candidate in candidates:
        x1, y1, x2, y2 = candidate.box
        p1 = (int(x1 * width), int(y1 * height))
        p2 = (int(x2 * width), int(y2 * height))
        color = (0, 255, 0) if candidate is selected else (0, 165, 255)
        cv2.rectangle(overlay, p1, p2, color, 3)
        cv2.putText(
            overlay,
            "{} det={:.3f} clip={:.3f}".format(
                query.target_text, candidate.detector_confidence, candidate.text_similarity
            ),
            (p1[0], max(20, p1[1] - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            color,
            2,
        )

    output_dir = HERE / "test_outputs" / args.output_name
    output_dir.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(output_dir / "detections.png"), overlay)
    report = {
        "test_scope": {
            "rgb_recognition": "real model inference",
            "aligned_depth": "simulated constant 2.0 m",
            "camera_info": "simulated calibrated pinhole model",
            "map_from_camera_tf": "simulated fixed transform",
            "ros_transport": "not run; ROS is unavailable on this machine",
        },
        "input": {
            "target_text": query.target_text,
            "base_class": query.base_class,
            "image": str(image_path),
            "image_size": [width, height],
            "source_frame": frame.source_frame,
            "depth_encoding": "16UC1",
            "depth_scale_to_m": 0.001,
            "map_from_camera": sensor_to_map,
        },
        "model": {
            "device": backend.device,
            "detector": "IDEA-Research/grounding-dino-tiny",
            "text_ranker": "openai/clip-vit-base-patch32",
        },
        "detections": [
            {
                "box_normalized_xyxy": [round(value, 6) for value in item.box],
                "detector_confidence": round(item.detector_confidence, 6),
                "clip_similarity": round(item.text_similarity, 6),
                "joint_confidence": round(item.joint_confidence, 6),
            }
            for item in candidates
        ],
        "selected_detection": None if selected is None else {
            "box_normalized_xyxy": [round(value, 6) for value in selected.box],
            "detector_confidence": round(selected.detector_confidence, 6),
            "clip_similarity": round(selected.text_similarity, 6),
            "joint_confidence": round(selected.joint_confidence, 6),
        },
        "updates": [update.to_dict() for update in sink.updates],
        "final_target": None if estimate is None else estimate.to_dict(),
        "passed": bool(candidates and estimate is not None),
    }
    (output_dir / "report.json").write_text(
        json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8"
    )
    print(json.dumps(report, indent=2, ensure_ascii=False))
    if not report["passed"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
