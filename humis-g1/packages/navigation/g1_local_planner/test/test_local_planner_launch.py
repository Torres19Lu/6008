"""Hardware-free checks for local_planner.launch and the config contract.

The planner math is covered by the C++ unit tests (costmap view, kinematic model,
path reference, MPC, local planner facade); these checks guard the launch wiring
and the param contract between the launch/YAML and the node.
"""
import os
import unittest
import xml.etree.ElementTree as ET

import yaml

PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LAUNCH  = os.path.join(PKG_DIR, "launch", "local_planner.launch")
CONFIG  = os.path.join(PKG_DIR, "config", "local_planner.yaml")


class TestLaunch(unittest.TestCase):
    def setUp(self):
        self.root = ET.parse(LAUNCH).getroot()

    def test_launch_is_well_formed(self):
        self.assertEqual(self.root.tag, "launch")

    def test_node_is_g1_local_planner(self):
        nodes = [n for n in self.root.iter("node")
                 if n.get("type") == "g1_local_planner_node"]
        self.assertEqual(len(nodes), 1)
        self.assertEqual(nodes[0].get("pkg"),  "g1_local_planner")
        self.assertEqual(nodes[0].get("name"), "g1_local_planner")

    def test_node_loads_config(self):
        node = next(n for n in self.root.iter("node")
                    if n.get("type") == "g1_local_planner_node")
        rosparams = [r for r in node.iter("rosparam")
                     if r.get("command") == "load"]
        self.assertTrue(any("local_planner.yaml" in (r.get("file") or "")
                            for r in rosparams))

    def test_optional_costmap_include_references_g1_costmap(self):
        incs = [i for i in self.root.iter("include")]
        self.assertTrue(any("g1_costmap" in (i.get("file") or "")
                            for i in incs))

    def test_optional_global_planner_include_references_g1_global_planner(self):
        incs = [i for i in self.root.iter("include")]
        self.assertTrue(any("g1_global_planner" in (i.get("file") or "")
                            for i in incs))

    def test_costmap_toggles_forwarded(self):
        # The costmap upstream toggles must be forwarded through the costmap include.
        inc = next(i for i in self.root.iter("include")
                   if "g1_costmap" in (i.get("file") or ""))
        passed = {a.get("name") for a in inc.iter("arg")}
        for name in ("start_slam", "start_frontend", "start_lidar",
                     "start_state", "map_path"):
            self.assertIn(name, passed)

    def test_topic_args_exposed_with_standalone_defaults(self):
        # The launch exposes cmd_topic/goal_topic so a coordinator (g1_nav) can route
        # this node via args; standalone defaults match the node's own defaults.
        args = {a.get("name"): a.get("default")
                for a in self.root if a.tag == "arg"}
        self.assertEqual(args.get("cmd_topic"),  "/cmd_vel")
        self.assertEqual(args.get("goal_topic"), "/move_base_simple/goal")

    def test_node_applies_topic_arg_overrides(self):
        # The node applies the args as <param> overrides (later param wins over the
        # YAML) so the coordinator's forwarded topics actually take effect.
        node = next(n for n in self.root.iter("node")
                    if n.get("type") == "g1_local_planner_node")
        params = {p.get("name"): p.get("value")
                  for p in node if p.tag == "param"}
        self.assertEqual(params.get("cmd_topic"),  "$(arg cmd_topic)")
        self.assertEqual(params.get("goal_topic"), "$(arg goal_topic)")


class TestConfig(unittest.TestCase):
    def setUp(self):
        with open(CONFIG) as f:
            self.cfg = yaml.safe_load(f)

    def test_map_frame(self):
        self.assertEqual(self.cfg["map_frame"], "map")

    def test_robot_base_frame(self):
        self.assertTrue(self.cfg["robot_base_frame"])

    def test_interfaces(self):
        self.assertEqual(self.cfg["path_topic"],       "/nav/global_path")
        self.assertEqual(self.cfg["costmap_topic"],    "/nav/local_costmap")
        self.assertEqual(self.cfg["goal_topic"],       "/move_base_simple/goal")
        self.assertEqual(self.cfg["cmd_topic"],        "/cmd_vel")
        self.assertEqual(self.cfg["local_plan_topic"], "/nav/local_plan")

    def test_rate_and_tolerance(self):
        self.assertGreater(self.cfg["update_rate"],         0.0)
        self.assertGreater(self.cfg["transform_tolerance"], 0.0)

    def test_tf_timeout_bounds_stale_pose(self):
        # a reused last-good TF older than tf_timeout -> STUCK (no tracking on a
        # frozen pose). Must exceed transform_tolerance (a fresh lookup is ~that old).
        self.assertGreater(self.cfg["tf_timeout"], self.cfg["transform_tolerance"])

    def test_enable_on_start_present(self):
        self.assertIn("enable_on_start", self.cfg)
        self.assertIn(self.cfg["enable_on_start"], (True, False))

    def test_horizon_and_dt(self):
        self.assertGreater(self.cfg["horizon"], 0)
        self.assertGreater(self.cfg["dt"],      0.0)

    def test_speed_limits(self):
        self.assertGreater(self.cfg["max_vx"],  0.0)
        self.assertLess(self.cfg["min_vx"],     0.0)
        self.assertGreater(self.cfg["max_vy"],  0.0)
        self.assertGreater(self.cfg["max_wz"],  0.0)

    def test_acc_limits(self):
        self.assertGreater(self.cfg["acc_lim_x"],     0.0)
        self.assertGreater(self.cfg["acc_lim_y"],     0.0)
        self.assertGreater(self.cfg["acc_lim_theta"], 0.0)

    def test_goal_tolerances(self):
        self.assertGreater(self.cfg["xy_goal_tolerance"],  0.0)
        self.assertGreater(self.cfg["yaw_goal_tolerance"], 0.0)

    def test_goal_align_radius(self):
        self.assertGreaterEqual(self.cfg["goal_align_radius"], 0.0)

    def test_reach_stall_radius_present(self):
        self.assertGreater(self.cfg["reach_stall_radius"], 0.0)

    def test_reach_goal_gap_present(self):
        self.assertGreaterEqual(self.cfg["reach_goal_gap"], 0.0)

    def test_cost_weights_present(self):
        for key in ("w_pos", "w_yaw", "w_obstacle", "w_effort",
                    "w_lateral", "w_rate", "w_pos_terminal", "w_yaw_terminal"):
            self.assertIn(key, self.cfg)
            self.assertGreaterEqual(self.cfg[key], 0.0)

    def test_obstacle_normalisation(self):
        self.assertGreater(self.cfg["lethal_cost"], 0.0)
        self.assertIn("treat_unknown_as_obstacle", self.cfg)
        self.assertIn(self.cfg["treat_unknown_as_obstacle"], (True, False))

    def test_v_ref(self):
        self.assertGreater(self.cfg["v_ref"], 0.0)

    def test_footprint_params_present(self):
        self.assertGreater(self.cfg["footprint_circle_radius"], 0.0)
        self.assertGreaterEqual(self.cfg["footprint_circle_count"], 1)
        self.assertGreater(self.cfg["footprint_lateral_width"], 0.0)
        self.assertGreater(self.cfg["footprint_forward_depth"], 0.0)

    def test_safety_filter_params_present(self):
        self.assertIn(self.cfg["safety_filter_enabled"], (True, False))
        self.assertGreater(self.cfg["safety_lethal_value"], 0)
        self.assertGreaterEqual(self.cfg["safety_brake_margin"], 0.0)
        self.assertGreaterEqual(self.cfg["safety_swept_subsamples"], 1)
        self.assertIn(self.cfg["safety_treat_unknown_as_obstacle"], (True, False))
        self.assertGreater(self.cfg["control_dt"], 0.0)

    def test_speed_regulation_params(self):
        # Accel alignment: deployed longitudinal accel matches g1_locomotion ax_max.
        self.assertEqual(self.cfg["acc_lim_x"], 0.5)
        # New speed-regulation knobs are present.
        for key in ("a_lat_max", "a_decel", "curv_lookahead",
                    "heading_slow_start", "heading_slow_full",
                    "heading_slow_floor", "v_min_move"):
            self.assertIn(key, self.cfg)
        # Approach decel must not exceed the executable longitudinal decel.
        self.assertLessEqual(self.cfg["a_decel"], self.cfg["acc_lim_x"])
        # heading_slow_full must exceed heading_slow_start (positive slowdown span).
        self.assertGreater(self.cfg["heading_slow_full"], self.cfg["heading_slow_start"])


if __name__ == "__main__":
    unittest.main()
