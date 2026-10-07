#!/usr/bin/env bash
# Real Unitree G1: D435 + Livox + SLAM + VLM + semantic navigation.
#
# Safety design:
#   * No Gazebo or simulation velocity bridge is started.
#   * g1_locomotion boots disarmed and this script never changes the G1 FSM.
#   * The VLM-to-navigation gate starts disabled and requires an operator service call.
#   * Automatic arm requires two explicit command-line flags.

set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_SETUP="${ROS_SETUP:-/opt/ros/noetic/setup.bash}"
HUMIS_G1_ROOT="${HUMIS_G1_ROOT:-${SCRIPT_DIR}/humis-g1}"
WS_SETUP="${WS_SETUP:-${HUMIS_G1_ROOT}/catkin_ws/devel/setup.bash}"
SLAM_DIR="${SLAM_DIR:-${SCRIPT_DIR}/slam}"
CONDA_SETUP="${CONDA_SETUP:-${HOME}/miniconda3/etc/profile.d/conda.sh}"
CONDA_ENV="${CONDA_ENV:-vlfm}"
VLFM_DIR="${VLFM_DIR:-${HOME}/桌面/vlfm}"
LOG_DIR="${LOG_DIR:-${SCRIPT_DIR}/logs/g1_real_navigation}"

TARGET="red chair"
BASE_CLASS="chair"
MAP_PATH="${G1_MAP_PATH:-}"
NETWORK_INTERFACE="${G1_NETWORK_INTERFACE:-}"
CAMERA_XYZ="${G1_D435_TO_CAMERA_XYZ:-}"
CAMERA_RPY="${G1_D435_TO_CAMERA_RPY:-}"
CAMERA_SERIAL=""
START_CAMERA="true"
RVIZ="true"
BUILD="true"
AUTO_ARM="false"
MOTION_CONFIRMED="false"

usage() {
    cat <<'EOF'
Usage:
  bash ./run_g1_red_chair_navigation.sh [options]

Required options or equivalent environment variables:
  --map-path PATH          Saved real-world map (G1_MAP_PATH)
  --interface NAME         Robot/Livox network interface (G1_NETWORK_INTERFACE)
  --camera-xyz "X Y Z"     Measured d435_link -> camera_link metres
                           (G1_D435_TO_CAMERA_XYZ)
  --camera-rpy "R P Y"     Measured d435_link -> camera_link radians
                           (G1_D435_TO_CAMERA_RPY)

Other options:
  --target TEXT            VLM target (default: "red chair")
  --base-class NAME        YOLO class (default: "chair")
  --camera-serial SERIAL   Select one RealSense camera
  --external-camera        Do not start realsense2_camera
  --auto-arm               Let g1_nav call /g1/arm after a goal is accepted
  --i-understand-motion    Required together with --auto-arm
  --no-rviz                Do not open RViz
  --skip-build             Skip incremental catkin build
  -h, --help               Show this help

The robot is never switched into walk_motion/FSM 500 by this script. Keep the
physical e-stop in hand. The semantic goal gate also starts disabled.
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --map-path)
            [[ $# -ge 2 ]] || { echo "--map-path requires a value" >&2; exit 2; }
            MAP_PATH="$2"; shift 2 ;;
        --interface)
            [[ $# -ge 2 ]] || { echo "--interface requires a value" >&2; exit 2; }
            NETWORK_INTERFACE="$2"; shift 2 ;;
        --camera-xyz)
            [[ $# -ge 2 ]] || { echo "--camera-xyz requires a value" >&2; exit 2; }
            CAMERA_XYZ="$2"; shift 2 ;;
        --camera-rpy)
            [[ $# -ge 2 ]] || { echo "--camera-rpy requires a value" >&2; exit 2; }
            CAMERA_RPY="$2"; shift 2 ;;
        --target)
            [[ $# -ge 2 ]] || { echo "--target requires a value" >&2; exit 2; }
            TARGET="$2"; shift 2 ;;
        --base-class)
            [[ $# -ge 2 ]] || { echo "--base-class requires a value" >&2; exit 2; }
            BASE_CLASS="$2"; shift 2 ;;
        --camera-serial)
            [[ $# -ge 2 ]] || { echo "--camera-serial requires a value" >&2; exit 2; }
            CAMERA_SERIAL="$2"; shift 2 ;;
        --external-camera) START_CAMERA="false"; shift ;;
        --auto-arm) AUTO_ARM="true"; shift ;;
        --i-understand-motion) MOTION_CONFIRMED="true"; shift ;;
        --no-rviz) RVIZ="false"; shift ;;
        --skip-build) BUILD="false"; shift ;;
        -h|--help) usage; exit 0 ;;
        *) echo "Unknown option: $1" >&2; usage >&2; exit 2 ;;
    esac
done

if [[ -z "${MAP_PATH}" || ! -d "${MAP_PATH}" ]]; then
    echo "A saved real map directory is required: --map-path PATH" >&2
    exit 2
fi
if [[ ! -f "${MAP_PATH}/manifest.yaml" ]]; then
    echo "Map has no manifest.yaml: ${MAP_PATH}" >&2
    exit 2
fi
if [[ -z "${NETWORK_INTERFACE}" ]]; then
    echo "Robot-facing network interface is required: --interface eth0" >&2
    exit 2
fi
if [[ -z "${CAMERA_XYZ}" || -z "${CAMERA_RPY}" ]]; then
    echo "Measured D435 mount XYZ and RPY are required; placeholder values are unsafe." >&2
    exit 2
fi
if [[ "${AUTO_ARM}" == "true" && "${MOTION_CONFIRMED}" != "true" ]]; then
    echo "--auto-arm requires --i-understand-motion and a physical e-stop." >&2
    exit 2
fi
if [[ ! "${TARGET}" =~ ^[A-Za-z0-9_-]+([[:space:]][A-Za-z0-9_-]+)*$ ]]; then
    echo "Target must currently be a simple English phrase, e.g. 'red chair'." >&2
    exit 2
fi
if [[ ! "${BASE_CLASS}" =~ ^[A-Za-z0-9_-]+$ ]]; then
    echo "Base class must be one YOLO class name, e.g. 'chair'." >&2
    exit 2
fi

validate_triplet() {
    local label="$1"
    local text="$2"
    local values=()
    read -r -a values <<< "${text}"
    if [[ ${#values[@]} -ne 3 ]]; then
        echo "${label} must contain exactly three numbers: ${text}" >&2
        exit 2
    fi
    local value
    for value in "${values[@]}"; do
        if [[ ! "${value}" =~ ^[-+]?([0-9]+([.][0-9]*)?|[.][0-9]+)([eE][-+]?[0-9]+)?$ ]]; then
            echo "${label} contains a non-numeric value: ${value}" >&2
            exit 2
        fi
    done
}

validate_triplet "camera XYZ" "${CAMERA_XYZ}"
validate_triplet "camera RPY" "${CAMERA_RPY}"

require_file() {
    [[ -f "$1" ]] || { echo "Missing $2: $1" >&2; exit 1; }
}
require_dir() {
    [[ -d "$1" ]] || { echo "Missing $2: $1" >&2; exit 1; }
}

require_file "${ROS_SETUP}" "ROS Noetic setup"
require_file "${CONDA_SETUP}" "Conda setup"
require_dir "${HUMIS_G1_ROOT}" "humis-g1 repository"
require_file "${SLAM_DIR}/ros_vlm_node.py" "realtime VLM node"
require_dir "${VLFM_DIR}/vlfm/vlm" "vlfm package"

source "${ROS_SETUP}"

if [[ "${BUILD}" == "true" ]]; then
    command -v catkin >/dev/null 2>&1 || {
        echo "catkin command not found. Install catkin_tools first." >&2
        exit 1
    }
    "${HUMIS_G1_ROOT}/scripts/link_workspace.sh"
    (
        cd "${HUMIS_G1_ROOT}/catkin_ws"
        catkin build \
            g1_description g1_lidar g1_locomotion \
            g1_slam_frontend g1_slam_backend \
            g1_costmap g1_global_planner g1_local_planner \
            g1_nav g1_semantic_nav
    )
fi

require_file "${WS_SETUP}" "catkin workspace setup"
source "${WS_SETUP}"

if [[ "${START_CAMERA}" == "true" ]] && ! rospack find realsense2_camera >/dev/null 2>&1; then
    echo "realsense2_camera is not installed or not visible in this ROS environment." >&2
    exit 1
fi

export G1_NETWORK_INTERFACE="${NETWORK_INTERFACE}"
export G1_D435_TO_CAMERA_XYZ="${CAMERA_XYZ}"
export G1_D435_TO_CAMERA_RPY="${CAMERA_RPY}"

mkdir -p "${LOG_DIR}"
declare -a COMPONENT_NAMES=()
declare -a COMPONENT_PIDS=()
CLEANED=0

quote() { printf '%q' "$1"; }

start_component() {
    local name="$1"
    local command="$2"
    local logfile="${LOG_DIR}/${name}.log"
    : > "${logfile}"
    setsid bash -lc "${command}" >"${logfile}" 2>&1 &
    local pid=$!
    COMPONENT_NAMES+=("${name}")
    COMPONENT_PIDS+=("${pid}")
    echo "[start] ${name} (PID ${pid}, log: ${logfile})"
}

cleanup() {
    if [[ "${CLEANED}" -eq 1 ]]; then return; fi
    CLEANED=1
    echo
    echo "[safety] Requesting G1 soft-stop before shutdown..."
    timeout 3 rosservice call /g1/halt >/dev/null 2>&1 || true

    local index
    for ((index=${#COMPONENT_PIDS[@]}-1; index>=0; index--)); do
        if kill -0 "${COMPONENT_PIDS[index]}" 2>/dev/null; then
            kill -TERM -- "-${COMPONENT_PIDS[index]}" 2>/dev/null || true
        fi
    done
    sleep 1
    for index in "${!COMPONENT_PIDS[@]}"; do
        if kill -0 "${COMPONENT_PIDS[index]}" 2>/dev/null; then
            kill -KILL -- "-${COMPONENT_PIDS[index]}" 2>/dev/null || true
        fi
    done
    wait 2>/dev/null || true
}

trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

show_log_tail() {
    echo "----- $1.log -----"
    tail -n 25 "${LOG_DIR}/$1.log" 2>/dev/null || true
}

wait_for_ros_topic() {
    local topic="$1" timeout_seconds="$2" started="${SECONDS}"
    while (( SECONDS - started < timeout_seconds )); do
        rostopic type "${topic}" >/dev/null 2>&1 && {
            echo "[ready] ${topic}"
            return 0
        }
        sleep 1
    done
    echo "Timed out waiting for ${topic}" >&2
    return 1
}

wait_for_port() {
    local port="$1" timeout_seconds="$2" started="${SECONDS}"
    while (( SECONDS - started < timeout_seconds )); do
        (echo >/dev/tcp/127.0.0.1/"${port}") >/dev/null 2>&1 && {
            echo "[ready] localhost:${port}"
            return 0
        }
        sleep 1
    done
    echo "Timed out waiting for localhost:${port}" >&2
    return 1
}

wait_for_tf() {
    local parent="$1" child="$2" timeout_seconds="$3" started="${SECONDS}"
    while (( SECONDS - started < timeout_seconds )); do
        local output
        output="$(timeout 3 rosrun tf tf_echo "${parent}" "${child}" 2>/dev/null || true)"
        if grep -q "Translation" <<< "${output}"; then
            echo "[ready] TF ${parent} -> ${child}"
            return 0
        fi
        sleep 1
    done
    echo "Timed out waiting for TF ${parent} -> ${child}" >&2
    return 1
}

ROS_Q="$(quote "${ROS_SETUP}")"
WS_Q="$(quote "${WS_SETUP}")"
CONDA_Q="$(quote "${CONDA_SETUP}")"
VLFM_Q="$(quote "${VLFM_DIR}")"
SLAM_Q="$(quote "${SLAM_DIR}")"
MAP_Q="$(quote "${MAP_PATH}")"
INTERFACE_Q="$(quote "${NETWORK_INTERFACE}")"
TARGET_Q="$(quote "${TARGET}")"
BASE_Q="$(quote "${BASE_CLASS}")"
SERIAL_Q="$(quote "${CAMERA_SERIAL}")"
OUTPUT_Q="$(quote "${LOG_DIR}/semantic_memory_realtime.json")"

echo "=== Real G1 semantic navigation ==="
echo "map        : ${MAP_PATH}"
echo "interface  : ${NETWORK_INTERFACE}"
echo "target     : ${TARGET}"
echo "auto arm   : ${AUTO_ARM}"
echo "goal gate  : disabled until operator enables it"
echo "logs       : ${LOG_DIR}"

MODEL_PREFIX="source ${CONDA_Q}; conda activate $(quote "${CONDA_ENV}"); cd ${VLFM_Q}; export PYTHONPATH=${VLFM_Q}; export LD_PRELOAD=\"\${CONDA_PREFIX}/lib/libstdc++.so.6\";"
start_component "openclip" "${MODEL_PREFIX} exec python -m vlfm.vlm.clipitm --port 12182"
start_component "yolov7" "${MODEL_PREFIX} exec python -m vlfm.vlm.yolov7 --port 12184"

start_component "g1_real_stack" "source ${ROS_Q}; source ${WS_Q}; exec roslaunch g1_semantic_nav semantic_navigation_real.launch map_path:=${MAP_Q} interface:=${INTERFACE_Q} expected_target:=${TARGET_Q} start_camera:=${START_CAMERA} camera_serial:=${SERIAL_Q} auto_arm:=${AUTO_ARM} rviz:=${RVIZ}"

if ! wait_for_ros_topic "/livox/lidar" 90; then
    show_log_tail "g1_real_stack"; exit 1
fi
# The status topic proves that the real /cmd_vel -> Unitree locomotion bridge is
# alive. Its presence does not arm the robot or change the robot FSM.
if ! wait_for_ros_topic "/g1/loco_status" 60; then
    show_log_tail "g1_real_stack"; exit 1
fi
if ! wait_for_ros_topic "/camera/color/image_raw" 90 ||
   ! wait_for_ros_topic "/camera/aligned_depth_to_color/image_raw" 90 ||
   ! wait_for_ros_topic "/camera/color/camera_info" 30; then
    show_log_tail "g1_real_stack"; exit 1
fi
if ! wait_for_ros_topic "/slam/map" 180 ||
   ! wait_for_ros_topic "/nav/costmap" 90 ||
   ! wait_for_ros_topic "/navigate_to/status" 30; then
    show_log_tail "g1_real_stack"; exit 1
fi
if ! wait_for_tf "map" "base_link" 180; then
    echo "SLAM has not localized the robot on the saved map." >&2
    show_log_tail "g1_real_stack"; exit 1
fi

if ! wait_for_port 12182 240; then show_log_tail "openclip"; exit 1; fi
if ! wait_for_port 12184 240; then show_log_tail "yolov7"; exit 1; fi

start_component "realtime_vlm" "source ${ROS_Q}; source ${WS_Q}; cd ${SLAM_Q}; exec /usr/bin/python3 ros_vlm_node.py _target:=${TARGET_Q} _base_class:=${BASE_Q} _process_rate:=1.0 _output:=${OUTPUT_Q}"

echo
echo "=== Sensors, SLAM, VLM and navigation are running ==="
echo "Automatic semantic navigation is still DISABLED."
echo
echo "Before allowing motion:"
echo "  1. Keep the physical e-stop in hand and clear the area."
echo "  2. Confirm /vlm/target_pose and /nav/costmap in RViz."
echo "  3. Put the G1 in balanced walk_motion / FSM 500 using the approved procedure."
if [[ "${AUTO_ARM}" == "false" ]]; then
    echo "  4. Arm ROS velocity streaming: rosservice call /g1/arm"
else
    echo "  4. g1_nav will arm automatically after accepting the goal."
fi
echo "  5. Open the VLM goal gate:"
echo "       rosservice call /semantic_goal_adapter/enable \"data: true\""
echo
echo "Emergency software stop: rosservice call /g1/halt"
echo "Press Ctrl-C to soft-stop and close all components."

while true; do
    sleep 2
    for index in "${!COMPONENT_PIDS[@]}"; do
        if ! kill -0 "${COMPONENT_PIDS[index]}" 2>/dev/null; then
            echo "Component stopped unexpectedly: ${COMPONENT_NAMES[index]}" >&2
            show_log_tail "${COMPONENT_NAMES[index]}"
            exit 1
        fi
    done
done
