"""Tests for the Livox Mid-360 config template and its launch-time generator."""
import importlib.util
import io
import json
import os
import subprocess
import sys
import tempfile
import unittest
from contextlib import redirect_stdout, redirect_stderr
from unittest import mock

PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TEMPLATE = os.path.join(PKG_DIR, "config", "MID360_config.json.template")
GEN_PATH = os.path.join(PKG_DIR, "scripts", "generate_livox_config.py")
PLACEHOLDER = "@HOST_IP@"


def _load_gen():
    spec = importlib.util.spec_from_file_location("generate_livox_config", GEN_PATH)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


class TestTemplate(unittest.TestCase):
    def test_template_is_valid_json(self):
        with open(TEMPLATE) as f:
            data = json.load(f)
        self.assertIn("MID360", data)

    def test_template_host_fields_are_placeholders(self):
        with open(TEMPLATE) as f:
            data = json.load(f)
        host = data["MID360"]["host_net_info"]
        for key in ("cmd_data_ip", "push_msg_ip", "point_data_ip", "imu_data_ip"):
            self.assertEqual(host[key], PLACEHOLDER)

    def test_template_lidar_ip_is_fixed_g1_value(self):
        with open(TEMPLATE) as f:
            data = json.load(f)
        self.assertEqual(data["lidar_configs"][0]["ip"], "192.168.123.120")


class TestRender(unittest.TestCase):
    def setUp(self):
        self.gen = _load_gen()
        with open(TEMPLATE) as f:
            self.template_text = f.read()

    def test_render_substitutes_all_host_fields(self):
        out = self.gen.render(self.template_text, "192.168.123.50")
        data = json.loads(out)
        host = data["MID360"]["host_net_info"]
        for key in ("cmd_data_ip", "push_msg_ip", "point_data_ip", "imu_data_ip"):
            self.assertEqual(host[key], "192.168.123.50")
        self.assertEqual(data["lidar_configs"][0]["ip"], "192.168.123.120")
        self.assertNotIn(PLACEHOLDER, out)

    def test_render_rejects_bad_ip(self):
        with self.assertRaises(ValueError):
            self.gen.render(self.template_text, "not-an-ip")

    def test_render_rejects_template_without_placeholder(self):
        with self.assertRaises(ValueError):
            self.gen.render('{"no": "placeholder"}', "192.168.123.50")


class TestResolveHostIp(unittest.TestCase):
    def setUp(self):
        self.gen = _load_gen()

    def test_parses_ip_command_output(self):
        sample = b"2: wlx0    inet 192.168.123.7/24 brd 192.168.123.255 scope global wlx0\n"
        with mock.patch.object(self.gen.subprocess, "check_output", return_value=sample):
            self.assertEqual(self.gen.resolve_host_ip("wlx0"), "192.168.123.7")

    def test_raises_when_no_inet(self):
        with mock.patch.object(self.gen.subprocess, "check_output", return_value=b""):
            with self.assertRaises(RuntimeError):
                self.gen.resolve_host_ip("wlx0")

    def test_raises_when_ip_command_fails(self):
        err = subprocess.CalledProcessError(1, "ip")
        with mock.patch.object(self.gen.subprocess, "check_output", side_effect=err):
            with self.assertRaises(RuntimeError):
                self.gen.resolve_host_ip("missing0")


class TestMain(unittest.TestCase):
    def setUp(self):
        self.gen = _load_gen()

    def test_host_ip_override_writes_file_and_prints_path(self):
        with tempfile.TemporaryDirectory() as d:
            out_path = os.path.join(d, "MID360_config.json")
            buf = io.StringIO()
            with redirect_stdout(buf):
                rc = self.gen.main([
                    "--template", TEMPLATE,
                    "--output", out_path,
                    "--host-ip", "192.168.123.99",
                ])
            self.assertEqual(rc, 0)
            # Exact match: stdout must carry NO trailing newline, because
            # roslaunch <param command> does not strip it and the Livox driver
            # would then fail to open a path ending in "\n".
            self.assertEqual(buf.getvalue(), out_path)
            with open(out_path) as f:
                data = json.load(f)
            self.assertEqual(
                data["MID360"]["host_net_info"]["point_data_ip"], "192.168.123.99")

    def test_bare_interface_with_host_ip_override(self):
        # Reproduces roslaunch tokenization when $(optenv G1_NETWORK_INTERFACE)
        # is empty but host_ip:= is given: the tokens become
        # "--interface --host-ip 192.168.123.42" (the empty interface value
        # collapses). The bare --interface must not swallow the --host-ip flag.
        with tempfile.TemporaryDirectory() as d:
            out_path = os.path.join(d, "MID360_config.json")
            buf = io.StringIO()
            with redirect_stdout(buf):
                rc = self.gen.main([
                    "--template", TEMPLATE,
                    "--output", out_path,
                    "--interface",
                    "--host-ip", "192.168.123.42",
                ])
            self.assertEqual(rc, 0)
            self.assertEqual(buf.getvalue().strip(), out_path)
            with open(out_path) as f:
                data = json.load(f)
            self.assertEqual(
                data["MID360"]["host_net_info"]["point_data_ip"], "192.168.123.42")

    def test_empty_host_ip_resolves_from_interface(self):
        fake_ip_output = b"2: lo    inet 127.0.0.1/8 scope host lo\n"
        with tempfile.TemporaryDirectory() as d:
            out_path = os.path.join(d, "MID360_config.json")
            buf = io.StringIO()
            with mock.patch.object(self.gen.subprocess, "check_output",
                                   return_value=fake_ip_output):
                with redirect_stdout(buf):
                    rc = self.gen.main([
                        "--template", TEMPLATE,
                        "--output", out_path,
                        "--interface", "lo",
                        "--host-ip", "",   # simulates roslaunch empty $(arg host_ip)
                    ])
            self.assertEqual(rc, 0)
            self.assertEqual(buf.getvalue().strip(), out_path)
            with open(out_path) as f:
                data = json.load(f)
            self.assertEqual(
                data["MID360"]["host_net_info"]["point_data_ip"], "127.0.0.1")

    def test_errors_when_no_host_ip_and_no_interface(self):
        with tempfile.TemporaryDirectory() as d:
            out_path = os.path.join(d, "MID360_config.json")
            errbuf = io.StringIO()
            with redirect_stderr(errbuf):
                rc = self.gen.main([
                    "--template", TEMPLATE,
                    "--output", out_path,
                    "--interface", "",
                    "--host-ip", "",
                ])
            self.assertEqual(rc, 2)
            self.assertFalse(os.path.exists(out_path))


if __name__ == "__main__":
    unittest.main()
