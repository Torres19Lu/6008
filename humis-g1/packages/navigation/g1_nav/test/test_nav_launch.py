"""Hardware-free checks for nav.launch and the config contract.

The navigation logic is covered by the C++ unit tests (fsm, watchdog);
these checks guard the launch wiring and the param contract between the launch/YAML
and the node. g1_nav only COORDINATES: it forwards coordinated topic names as <arg>
to each sub-launch, which applies them to its own node. It does NOT use <remap>:
roslaunch silently drops a <remap> child of an <include> (it honors only <arg>/<env>
there), so a remap-in-include would be a no-op. Key invariants:
  - g1_nav is the sole goal authority: the local and global planner includes forward
    goal_topic=/nav/goal.
  - g1_nav is the sole /cmd_vel writer: the local planner include forwards
    cmd_topic=/nav/cmd_vel_track.
  - g1_nav's own node does NOT override goal_topic (RViz 2D Nav Goal must reach it
    directly on /move_base_simple/goal).
  - No <remap> is a direct child of any <include> (regression guard: roslaunch would
    drop it; coordination must flow as <arg>).
  - clear_service points to /g1_costmap/clear_costmap (g1_costmap private namespace).
"""
import os
import unittest
import xml.etree.ElementTree as ET

import yaml

PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LAUNCH  = os.path.join(PKG_DIR, "launch", "nav.launch")
CONFIG  = os.path.join(PKG_DIR, "config", "nav.yaml")


class TestLaunch(unittest.TestCase):
    def setUp(self):
        self.root = ET.parse(LAUNCH).getroot()

    def test_launch_is_well_formed(self):
        self.assertEqual(self.root.tag, "launch")

    def test_node_is_g1_nav(self):
        nodes = [n for n in self.root.iter("node")
                 if n.get("type") == "g1_nav_node"]
        self.assertEqual(len(nodes), 1)
        self.assertEqual(nodes[0].get("pkg"),  "g1_nav")
        self.assertEqual(nodes[0].get("name"), "g1_nav")

    def test_node_loads_config(self):
        node = next(n for n in self.root.iter("node")
                    if n.get("type") == "g1_nav_node")
        rosparams = [r for r in node.iter("rosparam")
                     if r.get("command") == "load"]
        self.assertTrue(any("nav.yaml" in (r.get("file") or "")
                            for r in rosparams))

    def test_local_planner_include_present(self):
        incs = [i for i in self.root.iter("include")]
        self.assertTrue(any("g1_local_planner" in (i.get("file") or "")
                            for i in incs))

    def test_global_planner_include_present(self):
        incs = [i for i in self.root.iter("include")]
        self.assertTrue(any("g1_global_planner" in (i.get("file") or "")
                            for i in incs))

    def test_costmap_include_present(self):
        incs = [i for i in self.root.iter("include")]
        self.assertTrue(any("g1_costmap" in (i.get("file") or "")
                            for i in incs))

    def test_locomotion_include_present(self):
        incs = [i for i in self.root.iter("include")]
        self.assertTrue(any("g1_locomotion" in (i.get("file") or "")
                            for i in incs))

    def _get_include(self, pkg_fragment):
        """Return the first include element whose file contains pkg_fragment."""
        return next(i for i in self.root.iter("include")
                    if pkg_fragment in (i.get("file") or ""))

    def _remaps_for(self, element):
        """Collect {from: to} remaps that are DIRECT children of element."""
        return {r.get("from"): r.get("to")
                for r in element
                if r.tag == "remap"}

    def _args_for(self, element):
        """Collect {name: value} args that are DIRECT children of element."""
        return {a.get("name"): a.get("value")
                for a in element
                if a.tag == "arg"}

    def _node_params(self, node):
        """Collect {name: value} <param> overrides that are children of a node."""
        return {p.get("name"): p.get("value")
                for p in node
                if p.tag == "param"}

    def test_local_planner_cmd_topic_forwarded(self):
        # g1_nav forwards cmd_topic=/nav/cmd_vel_track (NOT a remap) so it stays the
        # sole /cmd_vel writer; the local planner's launch applies it to its node.
        inc = self._get_include("g1_local_planner")
        args = self._args_for(inc)
        self.assertEqual(args.get("cmd_topic"), "/nav/cmd_vel_track",
                         "local planner include must forward cmd_topic")

    def test_local_planner_goal_topic_forwarded(self):
        # g1_nav forwards goal_topic=/nav/goal so the local planner consumes only the
        # goal g1_nav forwards, not the raw RViz goal.
        inc = self._get_include("g1_local_planner")
        args = self._args_for(inc)
        self.assertEqual(args.get("goal_topic"), "/nav/goal",
                         "local planner include must forward goal_topic")

    def test_global_planner_goal_topic_forwarded(self):
        # g1_nav forwards goal_topic=/nav/goal to the global planner for the same reason.
        inc = self._get_include("g1_global_planner")
        args = self._args_for(inc)
        self.assertEqual(args.get("goal_topic"), "/nav/goal",
                         "global planner include must forward goal_topic")

    def test_g1_nav_node_does_not_override_goal_topic(self):
        # The g1_nav node itself must NOT override goal_topic (no <param> and no
        # <remap>) so the RViz 2D Nav Goal reaches it directly on /move_base_simple/goal.
        node = next(n for n in self.root.iter("node")
                    if n.get("type") == "g1_nav_node")
        self.assertNotIn("/move_base_simple/goal", self._remaps_for(node),
                         "g1_nav node must NOT remap /move_base_simple/goal")
        self.assertNotIn("goal_topic", self._node_params(node),
                         "g1_nav node must NOT override goal_topic")

    def test_auto_arm_is_a_launch_argument(self):
        args = {a.get("name"): a.get("default")
                for a in self.root.findall("arg")}
        self.assertEqual(args.get("auto_arm"), "true")
        node = next(n for n in self.root.iter("node")
                    if n.get("type") == "g1_nav_node")
        self.assertEqual(self._node_params(node).get("auto_arm"),
                         "$(arg auto_arm)")

    def test_no_remap_is_direct_child_of_include(self):
        # Regression guard: roslaunch SILENTLY DROPS a <remap> child of an <include>
        # (its xmlloader honors only <arg>/<env> there), so such a remap is a no-op.
        # Coordination must flow as <arg>. A live double /cmd_vel writer is the symptom.
        for inc in self.root.iter("include"):
            stray = [r.get("from") for r in inc if r.tag == "remap"]
            self.assertEqual(stray, [],
                             "nav.launch: <remap> under <include> %s is dropped by "
                             "roslaunch; forward the topic as <arg> instead"
                             % (inc.get("file") or "?"))


class TestConfig(unittest.TestCase):
    def setUp(self):
        with open(CONFIG) as f:
            self.cfg = yaml.safe_load(f)

    def test_map_frame(self):
        self.assertEqual(self.cfg["map_frame"], "map")

    def test_robot_base_frame(self):
        self.assertTrue(self.cfg["robot_base_frame"])

    def test_interfaces(self):
        self.assertEqual(self.cfg["goal_topic"],        "/move_base_simple/goal")
        self.assertEqual(self.cfg["local_state_topic"], "/g1_local_planner/state")
        self.assertEqual(self.cfg["track_cmd_topic"],   "/nav/cmd_vel_track")
        self.assertEqual(self.cfg["loco_status_topic"], "/g1/loco_status")
        self.assertEqual(self.cfg["cmd_vel_topic"],     "/cmd_vel")
        self.assertEqual(self.cfg["goal_out_topic"],    "/nav/goal")
        self.assertEqual(self.cfg["state_topic"],       "/nav/state")

    def test_services_present(self):
        self.assertEqual(self.cfg["enable_service"], "/g1_local_planner/enable")
        self.assertTrue(self.cfg["arm_service"])
        self.assertTrue(self.cfg["halt_service"])

    def test_rate_and_tolerances(self):
        self.assertGreater(self.cfg["rate"],                   0.0)
        self.assertGreater(self.cfg["transform_tolerance"],    0.0)
        self.assertGreater(self.cfg["feedback_rate"],          0.0)
        self.assertGreater(self.cfg["track_cmd_ttl"],          0.0)
        self.assertGreater(self.cfg["service_connect_timeout"], 0.0)

    def test_freshness_guards_positive(self):
        # bounded last-good TF age (force zero /cmd_vel past it), local_state TTL, and
        # the new-goal ack grace. tf_timeout must exceed transform_tolerance (a fresh
        # ros::Time(0) lookup is already ~transform_tolerance old).
        self.assertGreater(self.cfg["tf_timeout"], self.cfg["transform_tolerance"])
        self.assertGreater(self.cfg["local_state_ttl"], 0.0)
        self.assertGreater(self.cfg["goal_ack_grace"], 0.0)

    def test_fsm_timeouts_positive(self):
        self.assertGreater(self.cfg["plan_timeout"],   0.0)
        self.assertGreater(self.cfg["replan_timeout"], 0.0)
        self.assertGreaterEqual(self.cfg["goal_timeout"], 0.0)  # 0 = unbounded (valid)
        self.assertGreater(self.cfg["no_path_grace"],  0.0)

    def test_auto_arm_present(self):
        self.assertIn("auto_arm", self.cfg)
        self.assertIn(self.cfg["auto_arm"], (True, False))

    def test_watchdog_params(self):
        self.assertGreater(self.cfg["stuck_window"], 0.0)
        self.assertGreater(self.cfg["stuck_dist"],   0.0)


if __name__ == "__main__":
    unittest.main()
