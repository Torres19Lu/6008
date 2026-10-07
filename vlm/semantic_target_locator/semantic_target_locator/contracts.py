"""Stable interfaces between localization core and runtime-specific adapters."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Dict, Optional, Protocol, Sequence, Tuple


Point3 = Tuple[float, float, float]


class Status(str, Enum):
    IDLE = "IDLE"
    SEARCHING = "SEARCHING"
    NO_DETECTION = "NO_DETECTION"
    DEPTH_INVALID = "DEPTH_INVALID"
    TF_UNAVAILABLE = "TF_UNAVAILABLE"
    TARGET_DETECTED_3D = "TARGET_DETECTED_3D"
    TARGET_UNSTABLE = "TARGET_UNSTABLE"
    TARGET_LOCKED = "TARGET_LOCKED"
    TIMEOUT = "TIMEOUT"


@dataclass(frozen=True)
class TargetQuery:
    target_text: str
    base_class: str


@dataclass(frozen=True)
class Detection2D:
    class_name: str
    box: Tuple[float, float, float, float]
    detector_confidence: float
    text_similarity: float

    @property
    def joint_confidence(self) -> float:
        return self.detector_confidence * self.text_similarity


@dataclass(frozen=True)
class SynchronizedFrame:
    stamp: float
    source_frame: str
    width: int
    height: int
    rgb: Any = None
    depth: Any = None
    camera_info: Any = None
    metadata: Dict[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class Observation3D:
    label: str
    stamp: float
    position_map: Point3
    confidence: float
    num_depth_points: int


@dataclass(frozen=True)
class TargetEstimate:
    label: str
    position_map: Point3
    confidence: float
    observation_count: int
    position_spread_m: float

    def to_dict(self) -> Dict[str, Any]:
        return {
            "label": self.label,
            "frame_id": "map",
            "position": {
                "x": round(self.position_map[0], 3),
                "y": round(self.position_map[1], 3),
                "z": round(self.position_map[2], 3),
            },
            "confidence": round(self.confidence, 3),
            "observation_count": self.observation_count,
            "position_spread_m": round(self.position_spread_m, 3),
        }


@dataclass(frozen=True)
class Update:
    status: Status
    message: str
    target: Optional[TargetEstimate] = None

    def to_dict(self) -> Dict[str, Any]:
        result: Dict[str, Any] = {"status": self.status.value, "message": self.message}
        result["target"] = None if self.target is None else self.target.to_dict()
        return result


class FrameSource(Protocol):
    def next_frame(self, timeout_s: float) -> Optional[SynchronizedFrame]: ...


class VisionLanguageBackend(Protocol):
    def detect(self, frame: SynchronizedFrame, query: TargetQuery) -> Sequence[Detection2D]: ...


class DepthProvider(Protocol):
    def position_in_sensor(
        self, frame: SynchronizedFrame, detection: Detection2D
    ) -> Tuple[Point3, int]: ...


class TransformProvider(Protocol):
    def to_map(self, frame: SynchronizedFrame, point_sensor: Point3) -> Point3: ...


class ResultSink(Protocol):
    def publish(self, update: Update) -> None: ...
