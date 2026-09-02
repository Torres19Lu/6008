#!/usr/bin/env bash
# Install C++ math/solver dependencies not provided by ROS desktop-full:
#   - GTSAM 4.1 (pose-graph optimization, used by g1_slam_backend)
#   - OSQP + osqp-eigen (QP solver, used by g1_local_planner)
# Idempotent-ish: re-running re-installs/updates. Requires sudo.
set -euo pipefail

echo "[1/4] catkin_tools (catkin build)"
command -v catkin >/dev/null 2>&1 || sudo apt-get install -y python3-catkin-tools

echo "[2/4] GTSAM 4.1 (PPA)"
if ! dpkg -s libgtsam-dev >/dev/null 2>&1; then
  sudo add-apt-repository -y ppa:borglab/gtsam-release-4.1
  sudo apt-get update
  sudo apt-get install -y libgtsam-dev libgtsam-unstable-dev
fi

WORK="$(mktemp -d)"; trap 'rm -rf "${WORK}"' EXIT

echo "[3/4] OSQP (pinned v0.6.3)"
if [[ ! -f /usr/local/include/osqp/osqp.h && ! -f /usr/local/include/osqp.h ]]; then
  git clone --branch v0.6.3 --recursive https://github.com/osqp/osqp.git "${WORK}/osqp"
  cmake -S "${WORK}/osqp" -B "${WORK}/osqp/build" -DCMAKE_BUILD_TYPE=Release
  cmake --build "${WORK}/osqp/build" -j"$(nproc)"
  sudo cmake --install "${WORK}/osqp/build"
  sudo ldconfig
fi

echo "[4/4] osqp-eigen (pinned v0.8.0)"
if [[ ! -d /usr/local/include/OsqpEigen ]]; then
  git clone --branch v0.8.0 https://github.com/robotology/osqp-eigen.git "${WORK}/osqp-eigen"
  cmake -S "${WORK}/osqp-eigen" -B "${WORK}/osqp-eigen/build" -DCMAKE_BUILD_TYPE=Release
  cmake --build "${WORK}/osqp-eigen/build" -j"$(nproc)"
  sudo cmake --install "${WORK}/osqp-eigen/build"
  sudo ldconfig
fi

echo "[ok] system C++ deps installed"
