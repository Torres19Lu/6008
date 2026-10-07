"""Target-only localization service; contains no navigation or motion logic."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

try:
    from .contracts import (
        DepthProvider, FrameSource, Observation3D, ResultSink, Status,
        TargetEstimate, TargetQuery, TransformProvider, Update,
        VisionLanguageBackend,
    )
    from .memory import ObservationMemory
except ImportError:
    from contracts import (
        DepthProvider, FrameSource, Observation3D, ResultSink, Status,
        TargetEstimate, TargetQuery, TransformProvider, Update,
        VisionLanguageBackend,
    )
    from memory import ObservationMemory


@dataclass(frozen=True)
class LocatorConfig:
    max_frames: int = 120
    frame_timeout_s: float = 0.5
    detector_confidence: float = 0.25
    text_similarity: float = 0.2
    min_observations: int = 3
    association_radius_m: float = 0.6
    max_position_spread_m: float = 0.25


class SemanticTargetLocator:
    def __init__(
        self,
        frames: FrameSource,
        vlm: VisionLanguageBackend,
        depth: DepthProvider,
        transforms: TransformProvider,
        sink: ResultSink,
        config: LocatorConfig = LocatorConfig(),
    ) -> None:
        self.frames = frames
        self.vlm = vlm
        self.depth = depth
        self.transforms = transforms
        self.sink = sink
        self.config = config

    def locate(self, query: TargetQuery) -> Optional[TargetEstimate]:
        memory = ObservationMemory(
            self.config.min_observations,
            self.config.association_radius_m,
            self.config.max_position_spread_m,
        )
        self.sink.publish(Update(Status.SEARCHING, "search started"))
        last_stamp = float("-inf")

        for _ in range(self.config.max_frames):
            frame = self.frames.next_frame(self.config.frame_timeout_s)
            if frame is None:
                continue
            if frame.stamp <= last_stamp:
                continue
            last_stamp = frame.stamp

            detections = [
                detection for detection in self.vlm.detect(frame, query)
                if detection.class_name.casefold() == query.base_class.casefold()
                and detection.detector_confidence >= self.config.detector_confidence
                and detection.text_similarity >= self.config.text_similarity
            ]
            if not detections:
                self.sink.publish(Update(Status.NO_DETECTION, "no matching 2-D candidate"))
                continue
            detection = max(detections, key=lambda item: item.joint_confidence)

            try:
                point_sensor, num_points = self.depth.position_in_sensor(frame, detection)
            except ValueError as error:
                self.sink.publish(Update(Status.DEPTH_INVALID, str(error)))
                continue
            try:
                point_map = self.transforms.to_map(frame, point_sensor)
            except (KeyError, LookupError, ValueError) as error:
                self.sink.publish(Update(Status.TF_UNAVAILABLE, str(error)))
                continue

            memory.add(Observation3D(
                label=query.target_text,
                stamp=frame.stamp,
                position_map=point_map,
                confidence=detection.joint_confidence,
                num_depth_points=num_points,
            ))
            estimate = memory.estimate()
            if estimate is None:
                self.sink.publish(Update(
                    Status.TARGET_DETECTED_3D,
                    "3-D observation accepted; waiting for multi-frame lock",
                ))
                continue
            self.sink.publish(Update(Status.TARGET_LOCKED, "target position locked", estimate))
            return estimate

        self.sink.publish(Update(Status.TIMEOUT, "target was not locked before timeout"))
        return None
