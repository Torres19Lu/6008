"""Hardware-free checks for global_planner.launch and the config contract.

The planner math is covered by the C++ unit tests (grid, cost model, A*,
smoother, planner facade); these checks guard the launch wiring and the param
contract between the launch/YAML and the node.
"""
import os
import unittest
import xml.etree.ElementTree as ET

import yaml

PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LAUNCH  = os.path.join(PKG_DIR, "launch", "global_planner.launch")
CONFIG  = os.path.join(PKG_DIR, "config", "global_planner.yaml")


class TestLaunch(unittest.TestCase):
    def setUp(self):
        self.root = ET.parse(LAUNCH).getroot()

    def test_launch_is_well_formed(self):
        self.assertEqual(self.root.tag, "launch")

    def test_node_is_g1_global_planner(self):
        nodes = [n for n in self.root.iter("node")
                 if n.get("type") == "g1_global_planner_node"]
        self.assertEqual(len(nodes), 1)
        self.assertEqual(nodes[0].get("pkg"),  "g1_global_planner")
        self.assertEqual(nodes[0].get("name"), "g1_global_planner")

    def test_node_loads_config(self):
        node = next(n for n in self.root.iter("node")
                    if n.get("type") == "g1_global_planner_node")
        rosparams = [r for r in node.iter("rosparam")
                     if r.get("command") == "load"]
        self.assertTrue(any("global_planner.yaml" in (r.get("file") or "")
                            for r in rosparams))

    def test_optional_costmap_include_references_g1_costmap(self):
        incs = [i for i in self.root.iter("include")]
        self.assertTrue(any("g1_costmap" in (i.get("file") or "")
                            for i in incs))

    def test_costmap_toggles_forwarded(self):
        # The costmap upstream toggles must be forwarded through the include.
        inc = next(i for i in self.root.iter("include")
                   if "g1_costmap" in (i.get("file") or ""))
        passed = {a.get("name") for a in inc.iter("arg")}
        for name in ("start_slam", "start_frontend", "start_lidar",
                     "start_state", "map_path"):
            self.assertIn(name, passed)

    def test_goal_topic_arg_exposed_with_standalone_default(self):
        # The launch exposes goal_topic so a coordinator (g1_nav) can route this node
        # via an arg; the standalone default matches the node's own default.
        args = {a.get("name"): a.get("default")
                for a in self.root if a.tag == "arg"}
        self.assertEqual(args.get("goal_topic"), "/move_base_simple/goal")

    def test_node_applies_goal_topic_override(self):
        # The node applies the arg as a <param> override (later param wins over the
        # YAML) so the coordinator's forwarded goal topic actually takes effect.
        node = next(n for n in self.root.iter("node")
                    if n.get("type") == "g1_global_planner_node")
        params = {p.get("name"): p.get("value")
                  for p in node if p.tag == "param"}
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
        self.assertEqual(self.cfg["costmap_topic"], "/nav/costmap")
        self.assertEqual(self.cfg["goal_topic"],    "/move_base_simple/goal")
        self.assertEqual(self.cfg["path_topic"],    "/nav/global_path")

    def test_rate_and_tolerance(self):
        self.assertGreater(self.cfg["update_rate"],         0.0)
        self.assertGreater(self.cfg["transform_tolerance"], 0.0)

    def test_tf_and_replan_guards(self):
        # a reused last-good start TF older than tf_timeout -> skip planning (no
        # planning from a frozen start). Must exceed transform_tolerance.
        self.assertGreater(self.cfg["tf_timeout"], self.cfg["transform_tolerance"])
        # tolerate a few transient same-goal failures before wiping the latched path.
        self.assertGreaterEqual(self.cfg["replan_fail_tolerance"], 0)

    def test_lethal_threshold(self):
        self.assertGreaterEqual(self.cfg["lethal_threshold"], 1)
        self.assertLessEqual(self.cfg["lethal_threshold"],    100)

    def test_allow_unknown(self):
        self.assertIn(self.cfg["allow_unknown"], (True, False))

    def test_inflation_cost_weight(self):
        self.assertGreaterEqual(self.cfg["inflation_cost_weight"], 0.0)

    def test_heuristic_weight(self):
        self.assertGreaterEqual(self.cfg["heuristic_weight"], 1.0)

    def test_goal_snap_radius(self):
        self.assertGreaterEqual(self.cfg["goal_snap_radius"], 0.0)

    def test_max_expansions(self):
        self.assertGreaterEqual(self.cfg["max_expansions"], 0)

    def test_smooth_enable_present(self):
        self.assertIn("smooth_enable", self.cfg)

    def test_replan_on_block_present(self):
        self.assertIn("replan_on_block", self.cfg)


if __name__ == "__main__":
    unittest.main()
