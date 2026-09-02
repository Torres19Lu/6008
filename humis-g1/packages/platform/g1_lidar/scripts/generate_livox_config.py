#!/usr/bin/env python3
"""Generate the Livox Mid-360 driver config by injecting the PC1 host IP.

Launch-time host-IP injection: resolve the host (PC1) IPv4 from a network
interface (default $G1_NETWORK_INTERFACE) or an explicit --host-ip override,
substitute it into every '@HOST_IP@' placeholder in the tracked template, write
the result to --output, and print ONLY that output path to stdout so a roslaunch
<param command=...> can capture it as user_config_path.

Resolving at launch (not CMake configure time) means a DHCP/WiFi address change
needs no rebuild even when the DHCP/WiFi address changes. All diagnostics go to stderr; the
process returns non-zero on any failure so a bad config fails the launch loudly.
"""
import argparse
import os
import re
import subprocess
import sys

PLACEHOLDER = "@HOST_IP@"
_IPV4_RE = re.compile(r"^\d{1,3}(\.\d{1,3}){3}\Z")


def resolve_host_ip(interface):
    """Return the first IPv4 bound to *interface*, or raise RuntimeError."""
    try:
        out = subprocess.check_output(
            ["ip", "-4", "-o", "addr", "show", "dev", interface],
            stderr=subprocess.STDOUT,
        )
    except (OSError, subprocess.CalledProcessError) as exc:
        raise RuntimeError("failed to query interface '%s': %s" % (interface, exc))
    text = out.decode("utf-8", "replace")
    match = re.search(r"\binet\s+(\d+\.\d+\.\d+\.\d+)/", text)
    if not match:
        raise RuntimeError("no IPv4 address on interface '%s'" % interface)
    return match.group(1)


def render(template_text, host_ip):
    """Substitute *host_ip* for every placeholder in *template_text*."""
    if not _IPV4_RE.match(host_ip):
        raise ValueError("invalid host IP: %r" % host_ip)
    if PLACEHOLDER not in template_text:
        raise ValueError("template has no %s placeholder" % PLACEHOLDER)
    return template_text.replace(PLACEHOLDER, host_ip)


def main(argv=None):
    parser = argparse.ArgumentParser(description="Generate the Livox Mid-360 config.")
    parser.add_argument("--template", required=True, help="path to the .json.template")
    parser.add_argument("--output", required=True, help="path to write the generated json")
    parser.add_argument("--interface", nargs="?", default=os.environ.get("G1_NETWORK_INTERFACE", ""), const="",
                        help="network interface to resolve the host IP from")
    parser.add_argument("--host-ip", nargs="?", default="", const="",
                        help="explicit host IP override (skips interface resolution)")
    args = parser.parse_args(argv)

    host_ip = (args.host_ip or "").strip()
    if not host_ip:
        if not args.interface:
            sys.stderr.write(
                "g1_lidar: no --host-ip and G1_NETWORK_INTERFACE/--interface is empty\n")
            return 2
        try:
            host_ip = resolve_host_ip(args.interface)
        except RuntimeError as exc:
            sys.stderr.write("g1_lidar: %s\n" % exc)
            return 2

    try:
        with open(args.template) as f:
            rendered = render(f.read(), host_ip)
    except (OSError, ValueError) as exc:
        sys.stderr.write("g1_lidar: %s\n" % exc)
        return 2

    try:
        with open(args.output, "w") as f:
            f.write(rendered)
    except OSError as exc:
        sys.stderr.write("g1_lidar: cannot write %s: %s\n" % (args.output, exc))
        return 2

    # No trailing newline: roslaunch <param command> does NOT strip the captured
    # stdout, so a newline would become part of user_config_path and the Livox
    # driver would try to open a path ending in "\n" and fail to open the file.
    sys.stdout.write(args.output)
    return 0


if __name__ == "__main__":
    sys.exit(main())
