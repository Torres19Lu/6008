"""Select a free navigation pose near a semantic object.

This module deliberately has no ROS imports so its geometry and costmap handling
can be tested without a running ROS master.
"""

from dataclasses import dataclass
import math
from typing import Iterable, List, Optional, Sequence, Tuple


@dataclass(frozen=True)
class GridView:
    """Small immutable view of a nav_msgs/OccupancyGrid."""

    resolution: float
    width: int
    height: int
    origin_x: float
    origin_y: float
    origin_yaw: float
    data: Sequence[int]

    def value_at(self, world_x: float, world_y: float) -> Optional[int]:
        """Return the cell cost, or None when the point is outside the grid."""

        dx = world_x - self.origin_x
        dy = world_y - self.origin_y
        cosine = math.cos(self.origin_yaw)
        sine = math.sin(self.origin_yaw)
        local_x = cosine * dx + sine * dy
        local_y = -sine * dx + cosine * dy
        mx = int(math.floor(local_x / self.resolution))
        my = int(math.floor(local_y / self.resolution))
        if mx < 0 or my < 0 or mx >= self.width or my >= self.height:
            return None
        return int(self.data[my * self.width + mx])


@dataclass(frozen=True)
class ApproachCandidate:
    x: float
    y: float
    yaw: float
    cost: int
    radius: float
    angle_offset: float
    score: float


def _angle_distance(angle: float) -> float:
    return abs(math.atan2(math.sin(angle), math.cos(angle)))


def choose_approach_candidates(
    target_xy: Tuple[float, float],
    robot_xy: Tuple[float, float],
    grid: GridView,
    radii: Iterable[float] = (1.2, 1.0, 1.4),
    angle_offsets: Iterable[float] = (
        0.0,
        math.pi / 4.0,
        -math.pi / 4.0,
        math.pi / 2.0,
        -math.pi / 2.0,
        3.0 * math.pi / 4.0,
        -3.0 * math.pi / 4.0,
        math.pi,
    ),
    max_goal_cost: int = 50,
    cost_weight: float = 0.02,
    angle_weight: float = 0.25,
) -> List[ApproachCandidate]:
    """Return free approach poses ordered from most to least desirable.

    The primary point lies on the target-to-robot ray. Alternatives circle the
    target and are used when that point is occupied or the planner later reports
    that it is unreachable. Unknown cells and cells above max_goal_cost are
    rejected. The goal yaw always faces the semantic object.
    """

    target_x, target_y = target_xy
    robot_x, robot_y = robot_xy
    primary_angle = math.atan2(robot_y - target_y, robot_x - target_x)
    candidates = []

    for radius in radii:
        if radius <= 0.0:
            continue
        for offset in angle_offsets:
            angle = primary_angle + offset
            x = target_x + radius * math.cos(angle)
            y = target_y + radius * math.sin(angle)
            cost = grid.value_at(x, y)
            if cost is None or cost < 0 or cost > max_goal_cost:
                continue

            yaw = math.atan2(target_y - y, target_x - x)
            robot_distance = math.hypot(x - robot_x, y - robot_y)
            score = (
                robot_distance
                + cost_weight * float(cost)
                + angle_weight * _angle_distance(offset)
            )
            candidates.append(
                ApproachCandidate(
                    x=x,
                    y=y,
                    yaw=yaw,
                    cost=cost,
                    radius=radius,
                    angle_offset=offset,
                    score=score,
                )
            )

    return sorted(candidates, key=lambda candidate: candidate.score)
