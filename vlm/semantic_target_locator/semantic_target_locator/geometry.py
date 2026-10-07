"""Runtime-neutral depth and transform helpers."""

from __future__ import annotations

import math
import statistics
from typing import Any, Dict, Iterable, Sequence, Tuple

try:
    from .contracts import Detection2D, Point3, SynchronizedFrame
except ImportError:
    from contracts import Detection2D, Point3, SynchronizedFrame


def transform_point(matrix: Sequence[Sequence[float]], point: Point3) -> Point3:
    if matrix is None:
        raise LookupError("sensor_to_map transform is unavailable")
    if len(matrix) != 4 or any(len(row) != 4 for row in matrix):
        raise ValueError("transform must be a 4x4 matrix")
    vector = (point[0], point[1], point[2], 1.0)
    result = tuple(
        sum(float(matrix[row][column]) * vector[column] for column in range(4))
        for row in range(4)
    )
    if not math.isclose(result[3], 1.0, abs_tol=1e-6):
        raise ValueError("transform is not affine")
    return result[0], result[1], result[2]


class RegisteredPointDepthProvider:
    """Replay adapter for RGB-registered points stored in frame metadata."""

    def __init__(self, min_points: int = 3) -> None:
        self.min_points = min_points

    def position_in_sensor(
        self, frame: SynchronizedFrame, detection: Detection2D
    ) -> Tuple[Point3, int]:
        x1, y1, x2, y2 = detection.box
        selected = []
        for point in frame.metadata.get("registered_depth_points", []):
            u, v = float(point["u"]), float(point["v"])
            xyz = tuple(float(point[key]) for key in ("x", "y", "z"))
            if not all(math.isfinite(value) for value in xyz):
                continue
            if x1 * frame.width <= u <= x2 * frame.width and y1 * frame.height <= v <= y2 * frame.height:
                selected.append(xyz)
        if len(selected) < self.min_points:
            raise ValueError(
                "not enough valid depth points: {} < {}".format(len(selected), self.min_points)
            )
        median = tuple(statistics.median(axis) for axis in zip(*selected))
        return median, len(selected)  # type: ignore[return-value]


class MatrixTransformProvider:
    """Replay adapter for a per-frame ``sensor_to_map`` matrix."""

    def to_map(self, frame: SynchronizedFrame, point_sensor: Point3) -> Point3:
        return transform_point(frame.metadata["sensor_to_map"], point_sensor)


def detection_from_dict(value: Dict[str, Any]) -> Detection2D:
    return Detection2D(
        class_name=str(value["class_name"]),
        box=tuple(float(item) for item in value["box"]),  # type: ignore[arg-type]
        detector_confidence=float(value["detector_confidence"]),
        text_similarity=float(value["text_similarity"]),
    )
