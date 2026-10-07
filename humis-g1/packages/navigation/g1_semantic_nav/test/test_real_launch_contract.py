#!/usr/bin/env python3
"""Static safety contract for the real-G1 launch composition."""

from pathlib import Path
import unittest
import xml.etree.ElementTree as ET


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
LAUNCH_PATH = PACKAGE_ROOT / "launch" / "semantic_navigation_real.launch"


class RealLaunchContractTest(unittest.TestCase):
    def setUp(self):
        self.root = ET.parse(str(LAUNCH_PATH)).getroot()

    @staticmethod
    def include_args(include):
        return {
            element.get("name"): element.get("value")
            for element in include
            if element.tag == "arg"
        }

    def test_no_simulation_nodes_are_started(self):
        serialized = ET.tostring(self.root, encoding="unicode")
        self.assertNotIn("gazebo", serialized.lower())
        self.assertNotIn("sim_cmd_vel_bridge", serialized)

    def test_real_sensor_and_navigation_includes_exist(self):
        includes = list(self.root.iter("include"))
        self.assertTrue(any(
            "realsense2_camera" in (item.get("file") or "")
            for item in includes
        ))
        self.assertTrue(any(
            "g1_nav" in (item.get("file") or "")
            for item in includes
        ))

    def test_locomotion_is_started_once_through_state_chain(self):
        nav_include = next(
            item for item in self.root.iter("include")
            if "g1_nav" in (item.get("file") or "")
        )
        args = self.include_args(nav_include)
        self.assertEqual(args.get("start_state"), "true")
        self.assertEqual(args.get("start_locomotion"), "false")
        self.assertEqual(args.get("start_lidar"), "true")
        self.assertEqual(args.get("start_frontend"), "true")

    def test_semantic_goal_gate_starts_disabled(self):
        adapter = next(
            node for node in self.root.iter("node")
            if node.get("type") == "semantic_goal_adapter.py"
        )
        params = {
            element.get("name"): element.get("value")
            for element in adapter
            if element.tag == "param"
        }
        self.assertEqual(params.get("enabled_on_start"), "false")


if __name__ == "__main__":
    unittest.main()
