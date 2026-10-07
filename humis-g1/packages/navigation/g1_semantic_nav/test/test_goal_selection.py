#!/usr/bin/env python3

import math
import unittest

from navigation_tools import GridView, choose_approach_candidates


class GoalSelectionTest(unittest.TestCase):
    def make_grid(self, data=None):
        return GridView(
            resolution=0.5,
            width=20,
            height=20,
            origin_x=-5.0,
            origin_y=-5.0,
            origin_yaw=0.0,
            data=data if data is not None else [0] * 400,
        )

    def test_primary_candidate_is_between_robot_and_target(self):
        candidates = choose_approach_candidates(
            target_xy=(2.0, 0.0),
            robot_xy=(0.0, 0.0),
            grid=self.make_grid(),
            radii=(1.0,),
            angle_offsets=(0.0,),
        )
        self.assertEqual(len(candidates), 1)
        self.assertAlmostEqual(candidates[0].x, 1.0)
        self.assertAlmostEqual(candidates[0].y, 0.0)
        self.assertAlmostEqual(candidates[0].yaw, 0.0)

    def test_occupied_primary_uses_side_candidate(self):
        data = [0] * 400
        # World (1, 0) -> grid (12, 10).
        data[10 * 20 + 12] = 100
        candidates = choose_approach_candidates(
            target_xy=(2.0, 0.0),
            robot_xy=(0.0, 0.0),
            grid=self.make_grid(data),
            radii=(1.0,),
            angle_offsets=(0.0, math.pi / 2.0),
        )
        self.assertEqual(len(candidates), 1)
        self.assertAlmostEqual(candidates[0].x, 2.0)
        self.assertAlmostEqual(candidates[0].y, -1.0)

    def test_unknown_and_outside_cells_are_rejected(self):
        data = [-1] * 400
        candidates = choose_approach_candidates(
            target_xy=(2.0, 0.0),
            robot_xy=(0.0, 0.0),
            grid=self.make_grid(data),
            radii=(1.0,),
            angle_offsets=(0.0,),
        )
        self.assertEqual(candidates, [])


if __name__ == "__main__":
    unittest.main()
