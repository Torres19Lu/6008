#!/usr/bin/env bash
#
# filter_node_log.sh - run a node, dropping known-noisy log lines from its
# stdout/stderr while letting everything else (warnings, errors, status) through.
#
# Purpose: let a downstream launch quiet a noisy upstream driver WITHOUT patching
# it. The Livox-SDK2 console logger (spdlog) floods stdout with per-packet INFO
# lines under ROS1 - livox_ros_driver2 only calls DisableLivoxSdkConsoleLogger()
# under #ifdef BUILDING_ROS2 (lds_lidar.cpp), and that SDK switch is all-or-
# nothing anyway (it would also hide genuine SDK warnings/errors, which never
# reach /rosout). A line denylist drops just the steady-state spam.
#
# Used as a roslaunch launch-prefix:
#   launch-prefix="$(find g1_lidar)/scripts/filter_node_log.sh"
# The node command is appended as arguments.
#
# The deny pattern is an extended regex from $G1_LIDAR_LOG_FILTER; if unset or
# empty a code default is used (which the launch overrides). Lines matching it on
# EITHER stream are dropped. The node is exec'd so it stays the process roslaunch
# tracks - signals and the exit code are preserved for required="true".
#
set -u

# Code default; the launch overrides it via the env var. Steady-state Livox-SDK2
# spam: the device-detection heartbeat and the per-command ack.
DEFAULT_PATTERN='Handle detection data|Receive Command: Id'
PATTERN="${G1_LIDAR_LOG_FILTER:-}"
[ -z "${PATTERN}" ] && PATTERN="${DEFAULT_PATTERN}"

if [ "$#" -eq 0 ]; then
  echo "filter_node_log.sh: no command to run" >&2
  exit 2
fi

# Exec the node so it (not this shell) is the PID roslaunch monitors. Its
# stdout/stderr feed side-car greps via process substitution; when the node
# exits and closes the pipes the greps see EOF and exit on their own.
exec stdbuf -oL -eL "$@" \
  1> >(grep --line-buffered -Ev "${PATTERN}") \
  2> >(grep --line-buffered -Ev "${PATTERN}" >&2)
