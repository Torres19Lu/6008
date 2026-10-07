#!/usr/bin/env bash
# One-command Gazebo -> SLAM -> realtime VLM -> semantic navigation demo.

set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_SETUP="${ROS_SETUP:-/opt/ros/noetic/setup.bash}"
HUMIS_G1_ROOT="${HUMIS_G1_ROOT:-${SCRIPT_DIR}/humis-g1}"
SLAM_DIR="${SLAM_DIR:-${SCRIPT_DIR}/slam}"
WS_SETUP="${WS_SETUP:-${HUMIS_G1_ROOT}/catkin_ws/devel/setup.bash}"
CONDA_SETUP="${CONDA_SETUP:-${HOME}/miniconda3/etc/profile.d/conda.sh}"
CONDA_ENV="${CONDA_ENV:-vlfm}"
VLFM_DIR="${VLFM_DIR:-${HOME}/桌面/vlfm}"
TARGET="red chair"
BASE_CLASS="chair"
RVIZ="true"
BUILD="true"
LOG_DIR="${LOG_DIR:-${SCRIPT_DIR}/logs/red_chair_navigation}"

usage() {
    cat <<'EOF'
Usage:
  bash ./run_red_chair_navigation.sh [options]

Options:
  --target TEXT       Semantic target passed to VLM (default: "red chair")
  --base-class NAME   YOLO base class (default: "chair")
  --no-rviz           Do not open RViz
  --skip-build        Skip the incremental catkin build
  -h, --help          Show this help

Environment overrides:
  HUMIS_G1_ROOT  Path to humis-g1
  VLFM_DIR       Path to the vlfm repository
  CONDA_SETUP    Path to conda.sh
  CONDA_ENV      VLM conda environment (default: vlfm)
  ROS_SETUP      Path to ROS setup.bash
  WS_SETUP       Path to the catkin workspace setup.bash
  LOG_DIR        Output log directory

Example:
  VLFM_DIR="$HOME/桌面/vlfm" bash ./run_red_chair_navigation.sh
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --target)
            [[ $# -ge 2 ]] || { echo "--target requires a value" >&2; exit 2; }
            TARGET="$2"
            shift 2
            ;;
        --base-class)
            [[ $# -ge 2 ]] || { echo "--base-class requires a value" >&2; exit 2; }
            BASE_CLASS="$2"
            shift 2
            ;;
        --no-rviz)
            RVIZ="false"
            shift
            ;;
        --skip-build)
            BUILD="false"
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "Unknown option: $1" >&2
            usage >&2
            exit 2
            ;;
    esac
done

# The current YOLO/OpenCLIP pipeline was validated with English class names.
if [[ ! "${TARGET}" =~ ^[A-Za-z0-9_-]+([[:space:]][A-Za-z0-9_-]+)*$ ]]; then
    echo "Target must currently use a simple English phrase, e.g. 'red chair'." >&2
    exit 2
fi
if [[ ! "${BASE_CLASS}" =~ ^[A-Za-z0-9_-]+$ ]]; then
    echo "Base class must be a single YOLO class name, e.g. 'chair'." >&2
    exit 2
fi

require_file() {
    local path="$1"
    local description="$2"
    if [[ ! -f "${path}" ]]; then
        echo "Missing ${description}: ${path}" >&2
        exit 1
    fi
}

require_dir() {
    local path="$1"
    local description="$2"
    if [[ ! -d "${path}" ]]; then
        echo "Missing ${description}: ${path}" >&2
        exit 1
    fi
}

require_file "${ROS_SETUP}" "ROS Noetic setup"
require_file "${CONDA_SETUP}" "Conda setup"
require_dir "${HUMIS_G1_ROOT}" "humis-g1 repository"
require_dir "${SLAM_DIR}/g1_simulation" "g1_simulation package"
require_file "${SLAM_DIR}/ros_vlm_node.py" "realtime VLM node"
require_dir "${VLFM_DIR}/vlfm/vlm" "vlfm Python package"

source "${ROS_SETUP}"

if [[ "${BUILD}" == "true" ]]; then
    command -v catkin >/dev/null 2>&1 || {
        echo "catkin command not found. Install catkin_tools first." >&2
        exit 1
    }

    echo "[setup] Linking catkin packages..."
    "${HUMIS_G1_ROOT}/scripts/link_workspace.sh"

    # g1_simulation intentionally lives beside humis-g1. Add it to the generated
    # catkin workspace without moving or modifying the original package.
    SIM_LINK="${HUMIS_G1_ROOT}/catkin_ws/src/g1_simulation"
    if [[ -L "${SIM_LINK}" ]]; then
        if [[ "$(readlink -f "${SIM_LINK}")" != "$(readlink -f "${SLAM_DIR}/g1_simulation")" ]]; then
            echo "Existing g1_simulation link points elsewhere: ${SIM_LINK}" >&2
            exit 1
        fi
    elif [[ -e "${SIM_LINK}" ]]; then
        echo "Existing non-symlink path blocks g1_simulation link: ${SIM_LINK}" >&2
        exit 1
    else
        ln -s "${SLAM_DIR}/g1_simulation" "${SIM_LINK}"
        echo "[link] g1_simulation -> ${SLAM_DIR}/g1_simulation"
    fi

    echo "[build] Building incremental navigation/simulation packages..."
    (
        cd "${HUMIS_G1_ROOT}/catkin_ws"
        catkin build g1_simulation g1_nav g1_semantic_nav
    )
fi

require_file "${WS_SETUP}" "catkin workspace setup"
source "${WS_SETUP}"

mkdir -p "${LOG_DIR}"

declare -a COMPONENT_NAMES=()
declare -a COMPONENT_PIDS=()
CLEANED=0

quote() {
    printf '%q' "$1"
}

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

stop_group() {
    local pid="$1"
    if kill -0 "${pid}" 2>/dev/null; then
        kill -TERM -- "-${pid}" 2>/dev/null || true
    fi
}

cleanup() {
    if [[ "${CLEANED}" -eq 1 ]]; then
        return
    fi
    CLEANED=1
    echo
    echo "[stop] Stopping navigation demo..."
    local index
    for ((index=${#COMPONENT_PIDS[@]}-1; index>=0; index--)); do
        stop_group "${COMPONENT_PIDS[index]}"
    done

    # Give roslaunch and Gazebo a bounded grace period, then terminate only the
    # process groups created by this script.
    local attempt
    for attempt in {1..20}; do
        local any_alive=0
        for index in "${!COMPONENT_PIDS[@]}"; do
            if kill -0 "${COMPONENT_PIDS[index]}" 2>/dev/null; then
                any_alive=1
                break
            fi
        done
        [[ "${any_alive}" -eq 0 ]] && break
        sleep 0.25
    done
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
    local name="$1"
    echo "----- ${name}.log -----"
    tail -n 20 "${LOG_DIR}/${name}.log" 2>/dev/null || true
}

wait_for_ros_topic() {
    local topic="$1"
    local timeout_seconds="$2"
    local started="${SECONDS}"
    while (( SECONDS - started < timeout_seconds )); do
        if rostopic type "${topic}" >/dev/null 2>&1; then
            echo "[ready] ${topic}"
            return 0
        fi
        sleep 1
    done
    echo "Timed out waiting for ROS topic ${topic}" >&2
    return 1
}

wait_for_port() {
    local port="$1"
    local timeout_seconds="$2"
    local started="${SECONDS}"
    while (( SECONDS - started < timeout_seconds )); do
        if (echo >/dev/tcp/127.0.0.1/"${port}") >/dev/null 2>&1; then
            echo "[ready] localhost:${port}"
            return 0
        fi
        sleep 1
    done
    echo "Timed out waiting for localhost:${port}" >&2
    return 1
}

ROS_Q="$(quote "${ROS_SETUP}")"
WS_Q="$(quote "${WS_SETUP}")"
CONDA_Q="$(quote "${CONDA_SETUP}")"
VLFM_Q="$(quote "${VLFM_DIR}")"
SLAM_Q="$(quote "${SLAM_DIR}")"
TARGET_Q="$(quote "${TARGET}")"
BASE_Q="$(quote "${BASE_CLASS}")"
OUTPUT_Q="$(quote "${LOG_DIR}/semantic_memory_realtime.json")"

echo "=== G1 red-chair semantic navigation ==="
echo "target     : ${TARGET}"
echo "base class : ${BASE_CLASS}"
echo "logs       : ${LOG_DIR}"

MODEL_PREFIX="source ${CONDA_Q}; conda activate $(quote "${CONDA_ENV}"); cd ${VLFM_Q}; export PYTHONPATH=${VLFM_Q}; export LD_PRELOAD=\"\${CONDA_PREFIX}/lib/libstdc++.so.6\";"
start_component "openclip" "${MODEL_PREFIX} exec python -m vlfm.vlm.clipitm --port 12182"
start_component "yolov7" "${MODEL_PREFIX} exec python -m vlfm.vlm.yolov7 --port 12184"
start_component "gazebo" "source ${ROS_Q}; source ${WS_Q}; exec roslaunch g1_simulation g1_world.launch"

if ! wait_for_ros_topic "/camera/color/image_raw" 90 ||
   ! wait_for_ros_topic "/livox/lidar" 30; then
    show_log_tail "gazebo"
    exit 1
fi

start_component "slam_frontend" "source ${ROS_Q}; source ${WS_Q}; exec roslaunch g1_simulation slam_frontend_sim.launch"
if ! wait_for_ros_topic "/slam/frontend/odom" 90; then
    show_log_tail "slam_frontend"
    exit 1
fi

start_component "slam_backend" "source ${ROS_Q}; source ${WS_Q}; exec roslaunch g1_slam_backend slam_backend.launch"
if ! wait_for_ros_topic "/slam/map" 90; then
    show_log_tail "slam_backend"
    exit 1
fi

if ! wait_for_port 12182 240; then
    show_log_tail "openclip"
    exit 1
fi
if ! wait_for_port 12184 240; then
    show_log_tail "yolov7"
    exit 1
fi

start_component "realtime_vlm" "source ${ROS_Q}; source ${WS_Q}; cd ${SLAM_Q}; exec /usr/bin/python3 ros_vlm_node.py _target:=${TARGET_Q} _base_class:=${BASE_Q} _process_rate:=1.0 _output:=${OUTPUT_Q}"
sleep 2

start_component "semantic_navigation" "source ${ROS_Q}; source ${WS_Q}; exec roslaunch g1_semantic_nav semantic_navigation_sim.launch expected_target:=${TARGET_Q} rviz:=${RVIZ}"

if ! wait_for_ros_topic "/nav/costmap" 90; then
    show_log_tail "semantic_navigation"
    exit 1
fi
if ! wait_for_ros_topic "/navigate_to/status" 30; then
    show_log_tail "semantic_navigation"
    exit 1
fi

echo
echo "=== Full pipeline is running ==="
echo "The target is '${TARGET}'. No additional text input is required."
echo "VLM will publish a stable target, then navigation starts automatically."
echo "Press Ctrl-C to stop every component."
echo
echo "Useful checks:"
echo "  rostopic echo /vlm/semantic_target"
echo "  rostopic echo /vlm/target_pose"
echo "  rostopic echo /nav/state"
echo "  rostopic echo /cmd_vel"

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
