#!/usr/bin/env bash
# Single canonical entry point for remote_control. No Python / launch_ros /
# ament_index dependency — just `joy_node` (from ros-humble-joy) + our node.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
CONFIG="${REMOTE_CONTROL_CONFIG:-${SCRIPT_DIR}/config/xbox_like.yaml}"
NODE_BIN="${REPO_ROOT}/build/bin/remote_control_node"
ROS_SETUP="/opt/ros/humble/setup.bash"

[ -f "${ROS_SETUP}" ] || { echo "[rc] ROS2 Humble not found at ${ROS_SETUP}" >&2; exit 1; }
[ -x "${NODE_BIN}" ]  || { echo "[rc] 先运行 ./build.sh 构建 remote_control_node (${NODE_BIN})" >&2; exit 1; }
[ -f "${CONFIG}" ]    || { echo "[rc] 配置文件不存在: ${CONFIG}" >&2; exit 1; }

# Some ROS setup scripts probe optional env vars before defining them.
set +u
# shellcheck source=/dev/null
source "${ROS_SETUP}"
set -u

# joy_node 参数 inline，避免依赖上游 joy 包 share 目录里的 joy-params.yaml。
ros2 run joy joy_node --ros-args \
  -p device_id:=0 \
  -p deadzone:=0.05 \
  -p autorepeat_rate:=20.0 &
JOY_PID=$!
cleanup() {
  kill "${JOY_PID}" 2>/dev/null || true
  wait "${JOY_PID}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

exec "${NODE_BIN}" --ros-args --params-file "${CONFIG}"
