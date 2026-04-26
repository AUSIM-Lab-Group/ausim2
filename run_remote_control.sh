#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
OVERLAY_SETUP="${OVERLAY_SETUP:-${SCRIPT_DIR}/build/ros_ws/install/setup.bash}"
CONFIG_PATH="${CONFIG_PATH:-${SCRIPT_DIR}/third_party/ros_ws/src/remote_control/config/xbox_like.yaml}"
DEVICE_ID=""
USE_JOY="${USE_JOY:-true}"
DRY_RUN=0
MODE="gui"

log() {
  printf '[run_remote_control.sh] %s\n' "$*"
}

die() {
  printf '[run_remote_control.sh] %s\n' "$*" >&2
  exit 1
}

show_help() {
  cat <<EOF
用法:
  ./run_remote_control.sh [选项]

说明:
  一键启动 remote_control ROS2 遥控程序。
  脚本会先注入 ROS2 Humble 与仓内 overlay 环境，默认启动 Qt5 GUI 遥控窗口：
    - 默认同时拉起 joy_node 和 remote_control_gui_node
    - 有手柄且 /joy 有新消息时优先手柄；无 /joy 时回退 GUI 窗口键盘
  如需最低限度非 GUI 支持，传入 --headless：
    - 只启动 remote_control_node，保留前台终端键盘 stdin 控制

选项:
  --config <path>     指定 remote_control 参数 YAML
  --device-id <id>    指定 joy_node device_id；默认 0
  --use-joy <bool>    GUI 模式是否启动 joy_node；默认 true
  --headless          启动非 GUI 终端节点模式
  --gui               兼容旧参数；当前已是默认模式
  --dry-run           只打印将要执行的命令，不实际启动
  -h, --help          显示帮助

环境变量:
  ROS_SETUP           ROS2 setup.bash 路径，默认 /opt/ros/humble/setup.bash
  OVERLAY_SETUP       仓内 overlay setup.bash 路径，默认 build/ros_ws/install/setup.bash
  CONFIG_PATH         默认参数 YAML 路径
  USE_JOY             GUI 模式是否启动 joy_node，默认 true
EOF
}

while [ "$#" -gt 0 ]; do
  case "$1" in
    --config)
      [ "$#" -ge 2 ] || die "--config 需要路径参数"
      CONFIG_PATH="$2"
      shift 2
      ;;
    --device-id)
      [ "$#" -ge 2 ] || die "--device-id 需要数字参数"
      DEVICE_ID="$2"
      shift 2
      ;;
    --use-joy)
      [ "$#" -ge 2 ] || die "--use-joy 需要 true 或 false"
      case "$2" in
        true|false)
          USE_JOY="$2"
          ;;
        *)
          die "--use-joy 只接受 true 或 false"
          ;;
      esac
      shift 2
      ;;
    --headless)
      MODE="headless"
      shift
      ;;
    --gui)
      MODE="gui"
      shift
      ;;
    --dry-run)
      DRY_RUN=1
      shift
      ;;
    -h|--help)
      show_help
      exit 0
      ;;
    *)
      die "未知参数：$1"
      ;;
  esac
done

[ -f "${ROS_SETUP}" ] || die "未找到 ROS2 环境：${ROS_SETUP}"
[ -f "${OVERLAY_SETUP}" ] || die "未找到仓内 ROS overlay：${OVERLAY_SETUP}，请先运行 ./build.sh"
[ -f "${CONFIG_PATH}" ] || die "未找到 remote_control 参数文件：${CONFIG_PATH}"

set +u
# shellcheck disable=SC1090
source "${ROS_SETUP}"
# shellcheck disable=SC1090
source "${OVERLAY_SETUP}"
set -u

if [ "${MODE}" = "gui" ]; then
  if [ -z "${DEVICE_ID}" ]; then
    DEVICE_ID="0"
  fi
  log "GUI 模式启动，use_joy=${USE_JOY}，device_id=${DEVICE_ID}"
  COMMAND=(ros2 launch remote_control remote_control_gui.launch.py "device_id:=${DEVICE_ID}" "config:=${CONFIG_PATH}" "use_joy:=${USE_JOY}")
else
  log "Headless 模式只启动 remote_control_node（键盘前台终端模式）"
  COMMAND=(ros2 run remote_control remote_control_node --ros-args --params-file "${CONFIG_PATH}")
fi

if [ "${DRY_RUN}" -eq 1 ]; then
  printf '[run_remote_control.sh] 将执行:'
  printf ' %q' "${COMMAND[@]}"
  printf '\n'
  exit 0
fi

exec "${COMMAND[@]}"
