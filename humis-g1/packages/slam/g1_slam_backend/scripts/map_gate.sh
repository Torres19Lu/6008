#!/usr/bin/env bash
# Map gate: save / load / relocalize acceptance helpers for g1_slam_backend.
#
# These wrap the runtime services + the map->odom TF so the gate is repeatable.
# They do NOT manage roscore/roslaunch/rosbag: you bring up the backend (+ the
# frontend + the bag) in other terminals. Run this in a terminal that sees the
# same ROS master. ASCII only; no hardcoded paths (override via env/args).
#
# Subcommands:
#   save  [MAP_DIR]      after the mapping pass: dump the map directory
#   watch [TIMEOUT_S]    block until map->odom locks; print it (PASS) or time out (FAIL)
#   status               print current map->odom (if any) and the /slam/map size
#   reloc                exercise /slam/relocalize: global (no guess), then guided
#                        (use_guess at the robot's current map pose if available)
#
# Env overrides:
#   MAP_DIR    (default /tmp/g1_slam_map)
#   MAP_FRAME  / ODOM_FRAME / BASE_FRAME (default map / odom / base_link)
#   SAVE_SRV   / RELOC_SRV (default /slam/save_map, /slam/relocalize)
#   MAP_TOPIC  (default /slam/map)
set -u

MAP_DIR="${MAP_DIR:-/tmp/g1_slam_map}"
MAP_FRAME="${MAP_FRAME:-map}"
ODOM_FRAME="${ODOM_FRAME:-odom}"
BASE_FRAME="${BASE_FRAME:-base_link}"
SAVE_SRV="${SAVE_SRV:-/slam/save_map}"
RELOC_SRV="${RELOC_SRV:-/slam/relocalize}"
MAP_TOPIC="${MAP_TOPIC:-/slam/map}"

# Block until the map->odom transform is available, then print it. Args: target,
# source, timeout. Exit 0 if found, 1 on timeout. Loop timeout uses wall time so
# it works under sim time (bag replay with --clock).
watch_lock() {
  python3 - "$MAP_FRAME" "$ODOM_FRAME" "${1:-30}" <<'PY'
import sys, time, math, rospy, tf2_ros
target, source, timeout = sys.argv[1], sys.argv[2], float(sys.argv[3])
rospy.init_node('slam_gate_watch', anonymous=True, disable_signals=True)
buf = tf2_ros.Buffer(); tf2_ros.TransformListener(buf)
t0 = time.time()
while time.time() - t0 < timeout and not rospy.is_shutdown():
    try:
        tr = buf.lookup_transform(target, source, rospy.Time(0), rospy.Duration(0.5))
        t, q = tr.transform.translation, tr.transform.rotation
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
        print('  %s->%s: t=(%.3f, %.3f, %.3f) yaw=%.3f rad' %
              (target, source, t.x, t.y, t.z, yaw))
        sys.exit(0)
    except Exception:
        time.sleep(0.3)
sys.exit(1)
PY
}

reloc_with_guess() {
  # Use the robot's current map pose (map->base_link) as the guess, if available.
  python3 - "$MAP_FRAME" "$BASE_FRAME" "$RELOC_SRV" <<'PY'
import sys, rospy, tf2_ros
import geometry_msgs.msg as gm
from g1_msgs.srv import Relocalize, RelocalizeRequest
mf, bf, srv = sys.argv[1], sys.argv[2], sys.argv[3]
rospy.init_node('slam_gate_reloc_guess', anonymous=True, disable_signals=True)
buf = tf2_ros.Buffer(); tf2_ros.TransformListener(buf)
guess = gm.Pose(); guess.orientation.w = 1.0
have = False
import time; t0 = time.time()
while time.time() - t0 < 3.0:
    try:
        tr = buf.lookup_transform(mf, bf, rospy.Time(0), rospy.Duration(0.5))
        t, q = tr.transform.translation, tr.transform.rotation
        guess.position.x, guess.position.y, guess.position.z = t.x, t.y, t.z
        guess.orientation = q; have = True; break
    except Exception:
        time.sleep(0.3)
print('  guess from %s->%s: %s' % (mf, bf, 'current pose' if have else 'identity (no tf)'))
rospy.wait_for_service(srv, timeout=5.0)
resp = rospy.ServiceProxy(srv, Relocalize)(RelocalizeRequest(initial_guess=guess, use_guess=True))
print('  use_guess=true -> success=%s msg="%s"' % (resp.success, resp.message))
sys.exit(0 if resp.success else 1)
PY
}

case "${1:-help}" in
  save)
    dir="${2:-$MAP_DIR}"
    echo "[gate] save_map -> $dir"
    rosservice call "$SAVE_SRV" "path: '$dir'"
    echo "[gate] map dir contents:"; ls -1 "$dir" 2>/dev/null | sed 's/^/    /'
    ;;
  watch)
    echo "[gate] waiting for map->odom lock (timeout ${2:-30}s)..."
    if watch_lock "${2:-30}"; then echo "[gate] PASS: map->odom locked"
    else echo "[gate] FAIL: no map->odom within timeout"; exit 1; fi
    ;;
  status)
    echo "[gate] map->odom now:"
    watch_lock 2 || echo "    (none yet - not localized)"
    echo "[gate] $MAP_TOPIC size:"
    timeout 5 rostopic echo -n1 "$MAP_TOPIC" 2>/dev/null | grep -E 'width|height' | sed 's/^/    /' \
      || echo "    (no map message)"
    ;;
  reloc)
    echo "[gate] /slam/relocalize global (use_guess=false):"
    rosservice call "$RELOC_SRV" "{use_guess: false, initial_guess: {orientation: {w: 1.0}}}"
    echo "[gate] lock after global acquire:"; watch_lock 5 || echo "    (no lock)"
    echo "[gate] /slam/relocalize guided (use_guess=true):"
    reloc_with_guess || true
    ;;
  *)
    grep '^#' "$0" | grep -v '^#!' | sed 's/^# \{0,1\}//'
    ;;
esac
