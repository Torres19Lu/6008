"""JSON adapters for development without ROS or robot hardware."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence

try:
    from .contracts import Detection2D, SynchronizedFrame, TargetQuery, Update
    from .geometry import detection_from_dict
except ImportError:
    from contracts import Detection2D, SynchronizedFrame, TargetQuery, Update
    from geometry import detection_from_dict


class ReplayFrameSource:
    def __init__(self, frames: Sequence[Dict[str, Any]]) -> None:
        self._frames = list(frames)
        self._index = 0

    def next_frame(self, timeout_s: float) -> Optional[SynchronizedFrame]:
        del timeout_s
        if self._index >= len(self._frames):
            return None
        value = self._frames[self._index]
        self._index += 1
        return SynchronizedFrame(
            stamp=float(value["stamp"]),
            source_frame=str(value.get("source_frame", "camera_color_optical_frame")),
            width=int(value["image_size"][0]),
            height=int(value["image_size"][1]),
            metadata={
                "detections": value.get("detections", []),
                "registered_depth_points": value.get("registered_depth_points", []),
                "sensor_to_map": value.get("sensor_to_map"),
            },
        )


class ReplayVlmBackend:
    def detect(self, frame: SynchronizedFrame, query: TargetQuery) -> Sequence[Detection2D]:
        del query
        return [detection_from_dict(item) for item in frame.metadata["detections"]]


class CollectingSink:
    def __init__(self) -> None:
        self.updates: List[Update] = []

    def publish(self, update: Update) -> None:
        self.updates.append(update)


def load_replay(path: Path) -> Dict[str, Any]:
    with path.open(encoding="utf-8") as file:
        return json.load(file)
