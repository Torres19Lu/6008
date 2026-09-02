"""Hardware-free checks for slam_backend.launch and the config contract.

The optimization math is covered by the C++ unit tests (pose graph, scan context,
backend e2e) and the live loopy-bag gate; these checks guard the launch wiring and
the param contract between the launch/YAML and the node.
"""
import os
import unittest
import xml.etree.ElementTree as ET

import yaml

PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LAUNCH = os.path.join(PKG_DIR, "launch", "slam_backend.launch")
CONFIG = os.path.join(PKG_DIR, "config", "slam_backend.yaml")


class TestLaunch(unittest.TestCase):
    def setUp(self):
        self.root = ET.parse(LAUNCH).getroot()

    def test_launch_is_well_formed(self):
        self.assertEqual(self.root.tag, "launch")

    def test_node_is_g1_slam_backend(self):
        nodes = [n for n in self.root.iter("node")
                 if n.get("type") == "g1_slam_backend_node"]
        self.assertEqual(len(nodes), 1)
        self.assertEqual(nodes[0].get("pkg"), "g1_slam_backend")
        self.assertEqual(nodes[0].get("name"), "g1_slam_backend")

    def test_node_loads_config(self):
        node = next(n for n in self.root.iter("node")
                    if n.get("type") == "g1_slam_backend_node")
        rosparams = [r for r in node.iter("rosparam")
                     if r.get("command") == "load"]
        self.assertTrue(any("slam_backend.yaml" in (r.get("file") or "")
                            for r in rosparams))

    def test_optional_frontend_include(self):
        # start_frontend must bring up the upstream g1_slam_frontend.
        incs = [i for i in self.root.iter("include")]
        self.assertTrue(any("g1_slam_frontend" in (i.get("file") or "")
                            for i in incs))

    def test_upstream_toggles_passed_through(self):
        # start_lidar / start_state are forwarded to the frontend include.
        inc = next(i for i in self.root.iter("include")
                   if "g1_slam_frontend" in (i.get("file") or ""))
        passed = {a.get("name") for a in inc.iter("arg")}
        self.assertIn("start_lidar", passed)
        self.assertIn("start_state", passed)

    def test_map_path_arg_threaded_to_node(self):
        # A map_path arg (default empty) is threaded into the node so a
        # saved map can be auto-loaded at startup (localization mode).
        args = {a.get("name"): a for a in self.root.findall("arg")}
        self.assertIn("map_path", args)
        self.assertEqual(args["map_path"].get("default", ""), "")
        node = next(n for n in self.root.iter("node")
                    if n.get("type") == "g1_slam_backend_node")
        params = {p.get("name"): p.get("value") for p in node.iter("param")}
        self.assertIn("map_path", params)
        self.assertEqual(params["map_path"], "$(arg map_path)")


class TestConfig(unittest.TestCase):
    def setUp(self):
        with open(CONFIG) as f:
            self.cfg = yaml.safe_load(f)

    def test_frames_are_map_and_odom(self):
        self.assertEqual(self.cfg["map_frame"], "map")
        self.assertEqual(self.cfg["odom_frame"], "odom")

    def test_consumes_frontend_topics(self):
        self.assertEqual(self.cfg["odom_topic"], "/slam/frontend/odom")
        self.assertEqual(self.cfg["cloud_topic"],
                         "/slam/frontend/cloud_registered")

    def test_scan_context_block(self):
        sc = self.cfg["sc"]
        self.assertGreater(sc["num_rings"], 0)
        self.assertGreater(sc["num_sectors"], 0)
        self.assertGreater(sc["max_range"], sc["min_range"])

    def test_icp_block(self):
        icp = self.cfg["icp"]
        self.assertGreater(icp["max_corr_dist"], 0.0)
        self.assertGreater(icp["max_iter"], 0)
        self.assertIn("loop_voxel", icp)  # re-voxel the loop submap before ICP

    def test_reloc_block(self):
        # Relocalization params (localization-only).
        reloc = self.cfg["reloc"]
        self.assertGreater(reloc["search_radius"], 0.0)
        self.assertGreaterEqual(reloc["lost_streak"], 1)
        self.assertGreaterEqual(reloc["acquire_stride"], 1)
        self.assertGreater(reloc["rate"], 0.0)
        self.assertGreater(reloc["track_rebuild_dist"], 0.0)
        self.assertGreater(reloc["min_inlier"], 0.0)
        self.assertLessEqual(reloc["min_inlier"], 1.0)
        self.assertGreater(reloc["inlier_dist"], 0.0)
        self.assertGreaterEqual(reloc["num_candidates"], 1)

    def test_loop_robustness_block(self):
        # Loop-closure robustness: a Cauchy kernel + a front-end consistency gate
        # keep false/degenerate loops (corridors, repetitive rooms) from distorting
        # the map. Loop sigmas are looser than odom (trust comes from the kernel).
        loop = self.cfg["loop"]
        self.assertGreaterEqual(loop["robust_c"], 0.0)
        self.assertGreater(loop["max_dt"], 0.0)
        self.assertGreaterEqual(loop["max_dt_rate"], 0.0)
        self.assertGreater(loop["max_dr"], 0.0)
        self.assertGreaterEqual(loop["max_dr_rate"], 0.0)
        self.assertGreater(self.cfg["noise"]["loop_trans"],
                           self.cfg["noise"]["odom_trans"])

    def test_service_names_are_absolute(self):
        # Services live under /slam regardless of node namespace.
        for key in ("save_map_service", "load_map_service",
                    "relocalize_service"):
            self.assertTrue(self.cfg[key].startswith("/slam/"),
                            "%s must be an absolute /slam/* name" % key)


if __name__ == "__main__":
    unittest.main()
