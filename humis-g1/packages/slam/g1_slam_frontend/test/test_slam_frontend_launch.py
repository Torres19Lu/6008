"""Hardware-free checks for slam_frontend.launch and the config contract.

The ported estimator itself is covered by the live gate (a real G1 Mid-360 bag);
these checks guard the launch wiring and the humis-g1 interface adaptation.
"""
import os
import unittest
import xml.etree.ElementTree as ET

import yaml

PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LAUNCH = os.path.join(PKG_DIR, "launch", "slam_frontend.launch")
CONFIG = os.path.join(PKG_DIR, "config", "slam_frontend.yaml")


class TestLaunch(unittest.TestCase):
    def setUp(self):
        self.root = ET.parse(LAUNCH).getroot()

    def test_launch_is_well_formed(self):
        self.assertEqual(self.root.tag, "launch")

    def test_node_is_g1_slam_frontend(self):
        nodes = [n for n in self.root.iter("node")
                 if n.get("type") == "g1_slam_frontend_node"]
        self.assertEqual(len(nodes), 1)
        self.assertEqual(nodes[0].get("pkg"), "g1_slam_frontend")
        self.assertEqual(nodes[0].get("name"), "g1_slam_frontend")

    def test_outputs_remapped_to_contract_topics(self):
        remaps = {r.get("from"): r.get("to") for r in self.root.iter("remap")}
        self.assertEqual(remaps.get("/Odometry"), "/slam/frontend/odom")
        self.assertEqual(remaps.get("/cloud_registered"),
                         "/slam/frontend/cloud_registered")

    def test_optional_lidar_include_uses_customsg(self):
        # When start_lidar is used, g1_lidar must be brought up in CustomMsg mode.
        incs = [i for i in self.root.iter("include")]
        self.assertTrue(any("g1_lidar" in (i.get("file") or "") for i in incs))
        xfer = [a for inc in incs for a in inc.iter("arg")
                if a.get("name") == "xfer_format"]
        self.assertTrue(any(a.get("value") == "1" for a in xfer))

    def test_rear_clip_arg_threaded_to_node(self):
        # The rear follow-sector clip is on by default and the arg overrides the YAML
        # value on the node, so a run can disable it at launch (rear_clip:=false).
        args = {a.get("name"): a for a in self.root.findall("arg")}
        self.assertIn("rear_clip", args)
        self.assertEqual(args["rear_clip"].get("default"), "true")
        node = next(n for n in self.root.iter("node")
                    if n.get("type") == "g1_slam_frontend_node")
        params = {p.get("name"): p.get("value") for p in node.iter("param")}
        self.assertEqual(params.get("rear_clip/enable"), "$(arg rear_clip)")


class TestConfig(unittest.TestCase):
    def setUp(self):
        with open(CONFIG) as f:
            self.cfg = yaml.safe_load(f)

    def test_frames_block_present(self):
        frames = self.cfg["frames"]
        self.assertEqual(frames["odom_frame"], "odom")
        self.assertEqual(frames["base_frame"], "base_link")
        self.assertEqual(frames["lidar_frame"], "mid360_link")

    def test_livox_custom_msg_path(self):
        # lidar_type 1 = Livox CustomMsg handler.
        self.assertEqual(self.cfg["preprocess"]["lidar_type"], 1)

    def test_extrinsic_is_fixed_constant(self):
        self.assertFalse(self.cfg["mapping"]["extrinsic_est_en"])
        self.assertEqual(len(self.cfg["mapping"]["extrinsic_T"]), 3)
        self.assertEqual(len(self.cfg["mapping"]["extrinsic_R"]), 9)

    def test_rear_clip_block(self):
        # Rear-sector range clip: default ON (operator-follow is the normal mode),
        # a valid sector width, and a positive range cap.
        rc = self.cfg["rear_clip"]
        self.assertTrue(rc["enable"])
        self.assertGreater(rc["sector_deg"], 0.0)
        self.assertLessEqual(rc["sector_deg"], 360.0)
        self.assertGreater(rc["max_range"], 0.0)


if __name__ == "__main__":
    unittest.main()
