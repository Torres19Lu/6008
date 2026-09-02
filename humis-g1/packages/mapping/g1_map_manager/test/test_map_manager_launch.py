"""Hardware-free checks for the launch wiring and the YAML param contract.

The map-management logic is covered by the C++ unit tests and the integration rostest;
these checks guard the launch composition and the config contract:
  - map_manager.launch ALWAYS brings up the manager + the keyframe overlay relay.
  - the costmap include points keyframes_topic at /slam/keyframes_filtered so edits
    reach the static layer.
  - every node param has a default in config/map_manager.yaml (the override file).
"""
import os
import unittest
import xml.etree.ElementTree as ET

import yaml

PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LAUNCH = os.path.join(PKG_DIR, "launch", "map_manager.launch")
EDIT_LAUNCH = os.path.join(PKG_DIR, "launch", "map_edit.launch")
CONFIG = os.path.join(PKG_DIR, "config", "map_manager.yaml")


class TestLaunch(unittest.TestCase):
    def setUp(self):
        self.root = ET.parse(LAUNCH).getroot()

    def test_launch_is_well_formed(self):
        self.assertEqual(self.root.tag, "launch")

    def test_all_launches_parse(self):
        for path in (LAUNCH, EDIT_LAUNCH):
            self.assertEqual(ET.parse(path).getroot().tag, "launch")

    def test_manager_node_present_and_loads_config(self):
        nodes = [n for n in self.root.iter("node")
                 if n.get("type") == "g1_map_manager_node"]
        self.assertEqual(len(nodes), 1)
        self.assertEqual(nodes[0].get("pkg"), "g1_map_manager")
        rosparams = [r for r in nodes[0].iter("rosparam")
                     if r.get("command") == "load"]
        self.assertTrue(any("map_manager.yaml" in (r.get("file") or "")
                            for r in rosparams))

    def test_keyframe_overlay_always_launched(self):
        nodes = [n for n in self.root.iter("node")
                 if n.get("type") == "g1_keyframe_overlay_node"]
        self.assertEqual(len(nodes), 1)

    def test_costmap_include_present(self):
        incs = [i for i in self.root.iter("include")]
        self.assertTrue(any("g1_costmap" in (i.get("file") or "") for i in incs))

    def test_nav_include_present(self):
        incs = [i for i in self.root.iter("include")]
        self.assertTrue(any("g1_nav" in (i.get("file") or "") for i in incs))

    def test_nav_include_forwards_planners(self):
        # g1_nav runs the navigation legs but needs its planners; the include must
        # forward start_local + start_global (else start_nav brings up a stack that
        # cannot plan or track).
        nav_inc = next(i for i in self.root.iter("include")
                       if "g1_nav" in (i.get("file") or ""))
        args = {a.get("name"): a.get("value")
                for a in nav_inc if a.tag == "arg"}
        self.assertEqual(args.get("start_local"), "$(arg start_local)")
        self.assertEqual(args.get("start_global"), "$(arg start_global)")
        # the costmap + slam are owned by this launch, not re-launched by g1_nav.
        self.assertEqual(args.get("start_costmap"), "false")
        self.assertEqual(args.get("start_slam"), "false")

    def test_costmap_keyframes_topic_points_at_filtered(self):
        params = {p.get("name"): p.get("value") for p in self.root.iter("param")}
        self.assertEqual(params.get("g1_costmap/keyframes_topic"),
                         "/slam/keyframes_filtered")


class TestConfig(unittest.TestCase):
    def setUp(self):
        with open(CONFIG) as f:
            self.cfg = yaml.safe_load(f)

    def test_maps_root_present(self):
        self.assertIn("maps_root", self.cfg)  # "" => the g1_maps share dir

    def test_rates_and_timeouts_positive(self):
        self.assertGreater(self.cfg["state_rate"], 0.0)
        self.assertGreaterEqual(self.cfg["transition_wait"], 0.0)
        self.assertGreater(self.cfg["reloc_seed_timeout"], 0.0)
        self.assertGreater(self.cfg["reloc_global_timeout"], 0.0)
        self.assertGreater(self.cfg["transition_timeout"], 0.0)

    def test_tolerances_positive(self):
        self.assertGreater(self.cfg["nearest_anchor_max_dist"], 0.0)
        self.assertGreater(self.cfg["delete_radius"], 0.0)

    def test_action_and_topic_names(self):
        self.assertEqual(self.cfg["map_nav_action"], "/map_nav/navigate_to")
        self.assertEqual(self.cfg["nav_action"], "/navigate_to")
        self.assertEqual(self.cfg["state_topic"], "/map_manager/state")
        self.assertEqual(self.cfg["keyframe_poses_topic"], "/slam/keyframe_poses")

    def test_backend_service_names(self):
        self.assertEqual(self.cfg["load_map_service"], "/slam/load_map")
        self.assertEqual(self.cfg["save_map_service"], "/slam/save_map")
        self.assertEqual(self.cfg["relocalize_service"], "/slam/relocalize")
        self.assertEqual(self.cfg["begin_incremental_service"],
                         "/slam/begin_incremental")
        self.assertEqual(self.cfg["reset_service"], "/slam/reset")


if __name__ == "__main__":
    unittest.main()
