#!/usr/bin/env bash
# Launch and manage the complete Gazebo + SLAM + realtime VLM demonstration.

set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_SETUP="${ROS_SETUP:-/opt/ros/noetic/setup.bash}"
HUMIS_G1_ROOT="${HUMIS_G1_ROOT:-/home/ruruka/下载/humis-g1/humis-g1}"
WS_SETUP="${WS_SETUP:-${HUMIS_G1_ROOT}/catkin_ws/devel/setup.bash}"
CONDA_SETUP="${CONDA_SETUP:-${HOME}/miniconda3/etc/profile.d/conda.sh}"
VLFM_DIR="${VLFM_DIR:-${HOME}/桌面/vlfm}"
VLM_DIR="${VLM_DIR:-${SCRIPT_DIR}}"
CONDA_ENV="vlfm"
LOG_DIR="${VLM_DIR}/logs/realtime"

mkdir -p "${LOG_DIR}"
source "${ROS_SETUP}"
source "${WS_SETUP}"

declare -a NAMES=()
declare -a PIDS=()
APPROACH_PID=""
VIEWER_PID=""
CLEANED=0

start_component() {
    local name="$1"
    local command="$2"
    local logfile="${LOG_DIR}/${name}.log"
    : > "${logfile}"
    setsid bash -lc "${command}" >"${logfile}" 2>&1 &
    local pid=$!
    NAMES+=("${name}")
    PIDS+=("${pid}")
    echo "[start] ${name} (PID ${pid}, log: ${logfile})"
}

stop_group() {
    local pid="${1:-}"
    if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
        kill -TERM -- "-${pid}" 2>/dev/null || true
    fi
}

cleanup() {
    if [[ "${CLEANED}" -eq 1 ]]; then
        return
    fi
    CLEANED=1
    echo
    echo "[stop] 正在关闭实时演示的所有组件..."
    stop_group "${APPROACH_PID}"
    stop_group "${VIEWER_PID}"
    local index
    for ((index=${#PIDS[@]}-1; index>=0; index--)); do
        stop_group "${PIDS[index]}"
    done
    wait 2>/dev/null || true
    echo "[stop] 已全部关闭。"
}
trap cleanup EXIT
trap 'cleanup; exit 0' INT TERM HUP

wait_for_ros_topic() {
    local topic="$1"
    local timeout_s="$2"
    local elapsed=0
    until rostopic type "${topic}" >/dev/null 2>&1; do
        if (( elapsed >= timeout_s )); then
            echo "[error] 等待 topic 超时：${topic}"
            return 1
        fi
        sleep 1
        ((elapsed+=1))
    done
    echo "[ready] ${topic}"
}

wait_for_port() {
    local port="$1"
    local timeout_s="$2"
    local elapsed=0
    until (echo >/dev/tcp/127.0.0.1/"${port}") >/dev/null 2>&1; do
        if (( elapsed >= timeout_s )); then
            echo "[error] 等待模型端口超时：${port}"
            return 1
        fi
        sleep 1
        ((elapsed+=1))
    done
    echo "[ready] localhost:${port}"
}

show_log_tail() {
    local name="$1"
    echo "----- ${name}.log -----"
    tail -n 8 "${LOG_DIR}/${name}.log" 2>/dev/null || true
}

echo "=== G1 Gazebo + SLAM + Realtime VLM ==="
echo "日志目录：${LOG_DIR}"

# Start the expensive model loading in parallel with Gazebo initialization.
MODEL_PREFIX="source '${CONDA_SETUP}'; conda activate '${CONDA_ENV}'; cd '${VLFM_DIR}'; export PYTHONPATH='${VLFM_DIR}'; export LD_PRELOAD=\"\${CONDA_PREFIX}/lib/libstdc++.so.6\";"
start_component "openclip" "${MODEL_PREFIX} exec python -m vlfm.vlm.clipitm --port 12182"
start_component "yolov7" "${MODEL_PREFIX} exec python -m vlfm.vlm.yolov7 --port 12184"
start_component "gazebo" "source '${ROS_SETUP}'; source '${WS_SETUP}'; exec roslaunch g1_simulation g1_world.launch"

if ! wait_for_ros_topic "/camera/color/image_raw" 90; then
    show_log_tail "gazebo"
    exit 1
fi
if ! wait_for_ros_topic "/livox/lidar" 30; then
    show_log_tail "gazebo"
    exit 1
fi

start_component "slam_frontend" "source '${ROS_SETUP}'; source '${WS_SETUP}'; exec roslaunch g1_simulation slam_frontend_sim.launch"
if ! wait_for_ros_topic "/slam/frontend/odom" 90; then
    show_log_tail "slam_frontend"
    exit 1
fi

start_component "slam_backend" "source '${ROS_SETUP}'; source '${WS_SETUP}'; exec roslaunch g1_slam_backend slam_backend.launch"
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

start_component "realtime_vlm" "source '${ROS_SETUP}'; source '${WS_SETUP}'; cd '${VLM_DIR}'; exec /usr/bin/python3 ros_vlm_node.py _target:='red chair' _base_class:='chair' _process_rate:=1.0 _output:='${VLM_DIR}/semantic_memory_realtime.json'"
sleep 2

echo
echo "=== 全部组件已启动 ==="
echo "d : 复位机器人并运行走向红色椅子的演示"
echo "v : 打开/切换实时检测画面"
echo "l : 查看实时 VLM 最后几行日志"
echo "s : 显示关键 Topic 状态"
echo "q : 关闭全部组件并退出"

while true; do
    read -r -n 1 -p "请选择 [d/v/l/s/q]: " choice
    echo
    case "${choice}" in
        d|D)
            stop_group "${APPROACH_PID}"
            rosnode kill /approach_red_chair >/dev/null 2>&1 || true
            rosservice call /gazebo/set_model_state "model_state:
  model_name: 'g1_sim'
  pose:
    position: {x: -2.5, y: -2.0, z: 0.793}
    orientation: {x: 0.0, y: 0.0, z: 0.189, w: 0.982}
  twist:
    linear: {x: 0.0, y: 0.0, z: 0.0}
    angular: {x: 0.0, y: 0.0, z: 0.0}
  reference_frame: 'world'" >/dev/null
            setsid bash -lc "source '${ROS_SETUP}'; source '${WS_SETUP}'; exec rosrun g1_simulation approach_red_chair.py" >"${LOG_DIR}/approach.log" 2>&1 &
            APPROACH_PID=$!
            echo "[demo] 机器人已复位，3 秒后开始走向椅子。"
            ;;
        v|V)
            stop_group "${VIEWER_PID}"
            setsid bash -lc "source '${ROS_SETUP}'; source '${WS_SETUP}'; exec rqt_image_view /vlm/detection_image" >"${LOG_DIR}/viewer.log" 2>&1 &
            VIEWER_PID=$!
            echo "[viewer] 已打开 /vlm/detection_image"
            ;;
        l|L)
            show_log_tail "realtime_vlm"
            ;;
        s|S)
            for topic in /camera/color/image_raw /slam/frontend/odom /slam/map /vlm/semantic_target /vlm/target_pose; do
                type=$(rostopic type "${topic}" 2>/dev/null || true)
                if [[ -n "${type}" ]]; then
                    echo "[OK] ${topic} (${type})"
                else
                    echo "[--] ${topic}"
                fi
            done
            ;;
        q|Q)
            exit 0
            ;;
        *)
            echo "请输入 d、v、l、s 或 q。"
            ;;
    esac
done
