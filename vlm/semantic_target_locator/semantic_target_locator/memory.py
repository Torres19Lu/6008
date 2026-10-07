"""Robust multi-frame fusion for one requested semantic target."""

from __future__ import annotations

import math
import statistics
from typing import List, Optional

try:
    from .contracts import Observation3D, TargetEstimate
except ImportError:
    from contracts import Observation3D, TargetEstimate


class ObservationMemory:
    def __init__(self, min_observations: int, association_radius_m: float,
                 max_spread_m: float) -> None:
        self.min_observations = min_observations
        self.association_radius_m = association_radius_m
        self.max_spread_m = max_spread_m
        self._items: List[Observation3D] = []

    def add(self, observation: Observation3D) -> None:
        self._items.append(observation)

    def estimate(self) -> Optional[TargetEstimate]:
        if len(self._items) < self.min_observations:
            return None
        positions = [item.position_map for item in self._items]
        centre = tuple(statistics.median(axis) for axis in zip(*positions))
        inliers = [
            item for item in self._items
            if math.dist(item.position_map, centre) <= self.association_radius_m
        ]
        if len(inliers) < self.min_observations:
            return None
        fused = tuple(statistics.median(axis) for axis in zip(
            *(item.position_map for item in inliers)
        ))
        spread = max(math.dist(item.position_map, fused) for item in inliers)
        if spread > self.max_spread_m:
            return None
        return TargetEstimate(
            label=inliers[-1].label,
            position_map=fused,  # type: ignore[arg-type]
            confidence=statistics.fmean(item.confidence for item in inliers),
            observation_count=len(inliers),
            position_spread_m=spread,
        )
