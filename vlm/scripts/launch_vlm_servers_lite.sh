#!/usr/bin/env bash

set -euo pipefail

SESSION_NAME="${VLM_SESSION_NAME:-vlm_servers_lite}"
OPENCLIP_PORT="${OPENCLIP_PORT:-12182}"
YOLOV7_PORT="${YOLOV7_PORT:-12184}"

if [[ -z "${CONDA_PREFIX:-}" ]]; then
    echo "错误：请先执行 conda activate vlm"
    exit 1
fi

if ! command -v tmux >/dev/null 2>&1; then
    echo "错误：没有安装 tmux"
    exit 1
fi

VLFM_PYTHON="${VLFM_PYTHON:-$(command -v python)}"
LIBSTDCXX="${CONDA_PREFIX}/lib/libstdc++.so.6"
WORK_DIR="$(pwd)"

if [[ ! -f "${LIBSTDCXX}" ]]; then
    echo "错误：找不到 ${LIBSTDCXX}"
    exit 1
fi

if tmux has-session -t "${SESSION_NAME}" 2>/dev/null; then
    echo "模型服务已经存在：${SESSION_NAME}"
    echo "查看服务：tmux attach-session -t ${SESSION_NAME}"
    exit 0
fi

tmux new-session \
    -d \
    -s "${SESSION_NAME}" \
    -c "${WORK_DIR}"

tmux split-window \
    -h \
    -t "${SESSION_NAME}:0" \
    -c "${WORK_DIR}"

OPENCLIP_COMMAND="export LD_PRELOAD='${LIBSTDCXX}'; '${VLFM_PYTHON}' -m vlfm.vlm.clipitm --port ${OPENCLIP_PORT}"
YOLO_COMMAND="export LD_PRELOAD='${LIBSTDCXX}'; '${VLFM_PYTHON}' -m vlfm.vlm.yolov7 --port ${YOLOV7_PORT}"

tmux send-keys \
    -t "${SESSION_NAME}:0.0" \
    "${OPENCLIP_COMMAND}" \
    C-m

tmux send-keys \
    -t "${SESSION_NAME}:0.1" \
    "${YOLO_COMMAND}" \
    C-m

tmux select-layout \
    -t "${SESSION_NAME}:0" \
    even-horizontal

echo "已创建模型服务：${SESSION_NAME}"
echo "当前只启动 OpenCLIP 和 YOLOv7。"
echo "模型加载可能需要几十秒。"
echo
echo "查看模型服务："
echo "tmux attach-session -t ${SESSION_NAME}"
echo
echo "退出查看但保持服务运行："
echo "按 Ctrl+B，然后按 D"
echo
echo "结束两个模型服务："
echo "tmux kill-session -t ${SESSION_NAME}"
