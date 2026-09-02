"""Hardware-free checks for lidar.launch and its generator wiring."""
import json
import os
import subprocess
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET

PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LAUNCH = os.path.join(PKG_DIR, "launch", "lidar.launch")
TEMPLATE = os.path.join(PKG_DIR, "config", "MID360_config.json.template")
GEN = os.path.join(PKG_DIR, "scripts", "generate_livox_config.py")
FILTER = os.path.join(PKG_DIR, "scripts", "filter_node_log.sh")


class TestLaunch(unittest.TestCase):
    def setUp(self):
        self.tree = ET.parse(LAUNCH)
        self.root = self.tree.getroot()

    def test_launch_is_well_formed(self):
        self.assertEqual(self.root.tag, "launch")

    def test_user_config_path_uses_generator_command(self):
        params = [p for p in self.root.iter("param")
                  if p.get("name") == "user_config_path"]
        self.assertEqual(len(params), 1)
        cmd = params[0].get("command")
        self.assertIsNotNone(cmd)
        self.assertIn("generate_livox_config.py", cmd)
        self.assertIn("--template", cmd)
        self.assertIn("--output", cmd)

    def test_driver_node_present_and_pointcloud2(self):
        nodes = [n for n in self.root.iter("node")
                 if n.get("type") == "livox_ros_driver2_node"]
        self.assertEqual(len(nodes), 1)
        self.assertEqual(nodes[0].get("pkg"), "livox_ros_driver2")
        xfer = [a for a in self.root.iter("arg") if a.get("name") == "xfer_format"]
        self.assertEqual(xfer[0].get("default"), "0")

    def test_frame_id_default_is_mid360_link(self):
        frame = [a for a in self.root.iter("arg") if a.get("name") == "frame_id"]
        self.assertEqual(frame[0].get("default"), "mid360_link")

    def test_embedded_generator_call_produces_valid_config(self):
        with tempfile.TemporaryDirectory() as d:
            out_path = os.path.join(d, "MID360_config.json")
            rc = subprocess.call([
                sys.executable, GEN,
                "--template", TEMPLATE,
                "--output", out_path,
                "--host-ip", "192.168.123.77",
            ])
            self.assertEqual(rc, 0)
            with open(out_path) as f:
                data = json.load(f)
            self.assertEqual(
                data["MID360"]["host_net_info"]["cmd_data_ip"], "192.168.123.77")

    def test_quiet_driver_filter_is_wired(self):
        # The driver node is wrapped by the log filter (default on) so the
        # Livox-SDK2 console flood is dropped before it reaches the terminal,
        # while genuine warnings/errors pass through.
        args = {a.get("name"): a for a in self.root.findall("arg")}
        self.assertIn("quiet_driver", args)
        self.assertEqual(args["quiet_driver"].get("default"), "true")
        self.assertIn("driver_log_filter", args)
        node = next(n for n in self.root.iter("node")
                    if n.get("type") == "livox_ros_driver2_node")
        # launch-prefix is driven by an arg that points at the wrapper script.
        self.assertEqual(node.get("launch-prefix"), "$(arg driver_launch_prefix)")
        prefix_args = [a for a in self.root.iter("arg")
                       if a.get("name") == "driver_launch_prefix"]
        self.assertTrue(any("filter_node_log.sh" in (a.get("value") or "")
                            for a in prefix_args))
        # The deny pattern reaches the wrapper via the node environment.
        envs = {e.get("name"): e.get("value") for e in node.iter("env")}
        self.assertEqual(envs.get("G1_LIDAR_LOG_FILTER"), "$(arg driver_log_filter)")


class TestLogFilter(unittest.TestCase):
    """The filter_node_log.sh wrapper: drop the denylist, keep everything else."""

    def _run(self, emit, env=None):
        full_env = dict(os.environ)
        if env:
            full_env.update(env)
        # printf %s prints the (newline-bearing) argument verbatim; the wrapper
        # execs it and filters its stdout.
        return subprocess.run(
            [FILTER, "printf", "%s", emit],
            capture_output=True, text=True, env=full_env, timeout=15)

    def test_default_pattern_drops_flood_keeps_signal(self):
        emit = ("Handle detection data, handle:1, dev_type:9\n"
                "Receive Command: Id 258 Seq 9\n"
                "keep this status line\n"
                "[WARN] a real problem\n")
        # Stress it: the side-car greps flush asynchronously, so confirm the
        # captured output is stable run to run (no dropped/duplicated lines).
        for _ in range(15):
            out = self._run(emit).stdout
            self.assertNotIn("Handle detection data", out)
            self.assertNotIn("Receive Command: Id", out)
            self.assertIn("keep this status line", out)
            self.assertIn("[WARN] a real problem", out)

    def test_env_overrides_pattern(self):
        out = self._run("drop me now\nkeep this\n",
                        env={"G1_LIDAR_LOG_FILTER": "drop me"}).stdout
        self.assertNotIn("drop me now", out)
        self.assertIn("keep this", out)

    def test_no_command_is_an_error(self):
        rc = subprocess.run([FILTER], capture_output=True, text=True).returncode
        self.assertEqual(rc, 2)


if __name__ == "__main__":
    unittest.main()
