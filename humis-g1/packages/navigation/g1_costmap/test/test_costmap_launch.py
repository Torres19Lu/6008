"""Hardware-free checks for costmap.launch and the config contract.

The costmap math is covered by the C++ unit tests (grid, the three layers,
fusion) and the offline replay gate; these checks guard the launch wiring and
the param contract between the launch/YAML and the node.
"""
import os
import unittest
import xml.etree.ElementTree as ET

import yaml

PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LAUNCH = os.path.join(PKG_DIR, "launch", "costmap.launch")
CONFIG = os.path.join(PKG_DIR, "config", "costmap.yaml")


class TestLaunch(unittest.TestCase):
    def setUp(self):
        self.root = ET.parse(LAUNCH).getroot()

    def test_launch_is_well_formed(self):
        self.assertEqual(self.root.tag, "launch")

    def test_node_is_g1_costmap(self):
        nodes = [n for n in self.root.iter("node")
                 if n.get("type") == "g1_costmap_node"]
        self.assertEqual(len(nodes), 1)
        self.assertEqual(nodes[0].get("pkg"), "g1_costmap")
        self.assertEqual(nodes[0].get("name"), "g1_costmap")

    def test_node_loads_config(self):
        node = next(n for n in self.root.iter("node")
                    if n.get("type") == "g1_costmap_node")
        rosparams = [r for r in node.iter("rosparam")
                     if r.get("command") == "load"]
        self.assertTrue(any("costmap.yaml" in (r.get("file") or "")
                            for r in rosparams))

    def test_optional_slam_include(self):
        # start_slam must bring up the upstream g1_slam_backend.
        incs = [i for i in self.root.iter("include")]
        self.assertTrue(any("g1_slam_backend" in (i.get("file") or "")
                            for i in incs))

    def test_upstream_toggles_passed_through(self):
        # The backend's deeper toggles are forwarded through the include.
        inc = next(i for i in self.root.iter("include")
                   if "g1_slam_backend" in (i.get("file") or ""))
        passed = {a.get("name") for a in inc.iter("arg")}
        for name in ("start_frontend", "start_lidar", "start_state", "map_path"):
            self.assertIn(name, passed)


class TestConfig(unittest.TestCase):
    def setUp(self):
        with open(CONFIG) as f:
            self.cfg = yaml.safe_load(f)

    def test_frame_and_base(self):
        self.assertEqual(self.cfg["map_frame"], "map")
        self.assertTrue(self.cfg["lidar_frame"])  # dynamic origin + lidar-plane band top

    def test_interfaces(self):
        self.assertEqual(self.cfg["keyframes_topic"], "/slam/keyframes")
        self.assertEqual(self.cfg["cloud_topic"],
                         "/slam/frontend/cloud_registered")
        self.assertEqual(self.cfg["costmap_topic"], "/nav/costmap")
        self.assertEqual(self.cfg["costmap_raw_topic"], "/nav/costmap_raw")

    def test_grid_params(self):
        self.assertGreater(self.cfg["resolution"], 0.0)
        self.assertGreaterEqual(self.cfg["margin"], 0.0)
        self.assertGreater(self.cfg["update_rate"], 0.0)
        self.assertGreater(self.cfg["transform_tolerance"], 0.0)

    def test_ground_band(self):
        # Ground-relative band + lidar-plane top + ground-surface params.
        self.assertGreaterEqual(self.cfg["walkable_layers"], 0)
        self.assertGreater(self.cfg["obstacle_max_height"], 0.0)  # safety cap
        self.assertGreaterEqual(self.cfg["phantom_drop"], 0.0)
        # Foot sole contact geometry (replaces the old foot_thickness scalar).
        self.assertLess(self.cfg["foot_sole_z"], 0.0)  # sole below the ankle origin
        self.assertGreaterEqual(self.cfg["foot_sole_radius"], 0.0)
        self.assertGreater(self.cfg["foot_toe_x"], self.cfg["foot_heel_x"])  # toe ahead of heel
        self.assertGreaterEqual(self.cfg["foot_front_half_y"], 0.0)
        self.assertGreaterEqual(self.cfg["foot_rear_half_y"], 0.0)
        self.assertGreaterEqual(self.cfg["max_fill_radius"], 0.0)
        self.assertGreaterEqual(self.cfg["ground_min_points"], 1)

    def test_foot_frames(self):
        self.assertTrue(self.cfg["left_foot_frame"])
        self.assertTrue(self.cfg["right_foot_frame"])

    def test_dynamic_layer_block(self):
        self.assertGreater(self.cfg["dyn_max_range"], 0.0)
        self.assertIn("dyn_raycast", self.cfg)
        self.assertGreaterEqual(self.cfg["dyn_obstacle_frac"], 0.0)
        self.assertLessEqual(self.cfg["dyn_obstacle_frac"], 1.0)
        self.assertGreaterEqual(self.cfg["dyn_min_returns"], 1)
        # Bidirectional log-odds change evidence.
        self.assertGreater(self.cfg["dyn_prob_hit"], 0.5)
        self.assertLess(self.cfg["dyn_prob_miss"], 0.5)
        self.assertGreater(self.cfg["dyn_clamp_max"], self.cfg["dyn_clamp_min"])
        self.assertGreater(self.cfg["dyn_decay_half_life"], 0.0)
        self.assertIn("dyn_fill_thr", self.cfg)
        # Clearing a static obstacle is the HIGH bar (asymmetry).
        self.assertGreater(self.cfg["dyn_clear_thr"], self.cfg["dyn_add_thr"])
        # Change diagnostic topic + localization static freeze guard.
        self.assertEqual(self.cfg["costmap_change_prob_topic"], "/nav/change_prob")
        self.assertIn("freeze_static", self.cfg)

    def test_inflation_block(self):
        self.assertGreater(self.cfg["robot_radius"], 0.0)
        self.assertGreaterEqual(self.cfg["inflation_radius"],
                                self.cfg["robot_radius"])
        self.assertGreater(self.cfg["cost_scaling_factor"], 0.0)

    def test_static_and_normal_block(self):
        self.assertGreaterEqual(self.cfg["static_max_range"], 0.0)  # 0 = unlimited
        self.assertIn("use_normals", self.cfg)
        self.assertGreater(self.cfg["normal_k"], 0)
        self.assertGreater(self.cfg["ground_normal_angle"], 0.0)
        self.assertGreater(self.cfg["vertical_normal_angle"],
                           self.cfg["ground_normal_angle"])
        self.assertGreaterEqual(self.cfg["normal_flat_grace_layers"], 0)

    def test_local_window_block(self):
        self.assertIn("local_costmap_enable", self.cfg)
        self.assertEqual(self.cfg["local_costmap_topic"], "/nav/local_costmap")
        self.assertGreater(self.cfg["local_size_m"], 0.0)
        self.assertTrue(self.cfg["robot_base_frame"])

    def test_clear_service_block(self):
        # clear_costmap service keys must be present in the YAML.
        self.assertIn("clear_service_enable", self.cfg)
        self.assertIn("clear_service_name", self.cfg)
        self.assertTrue(self.cfg["clear_service_name"])  # non-empty string

    def test_probabilistic_block(self):
        self.assertGreater(self.cfg["prob_hit"], 0.5)
        self.assertLess(self.cfg["prob_miss"], 0.5)
        self.assertGreater(self.cfg["prob_clamp_max"], self.cfg["prob_clamp_min"])
        self.assertGreater(self.cfg["occupancy_thr"], 0.0)
        self.assertLess(self.cfg["occupancy_thr"], 1.0)
        self.assertIn("erode_obstacles", self.cfg)
        self.assertIn("angular_fill", self.cfg)
        self.assertGreater(self.cfg["angular_fill_step_deg"], 0.0)
        self.assertEqual(self.cfg["costmap_prob_topic"], "/nav/costmap_prob")


if __name__ == "__main__":
    unittest.main()
