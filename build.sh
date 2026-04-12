#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${SCRIPT_DIR}/build"
ROS_SETUP="/opt/ros/humble/setup.bash"

APT_BUILD_PACKAGES=(
  build-essential
  cmake
  libeigen3-dev
  libyaml-cpp-dev
  libglfw3-dev
)

ROS_CHECK_PACKAGES=(
  ros-humble-ament-cmake
  ros-humble-rclcpp
  ros-humble-geometry-msgs
  ros-humble-nav-msgs
  ros-humble-sensor-msgs
  ros-humble-rosgraph-msgs
  ros-humble-tf2
  ros-humble-tf2-ros
)

log() {
  printf '[build.sh] %s\n' "$*"
}

die() {
  printf '[build.sh] %s\n' "$*" >&2
  exit 1
}

show_help() {
  cat <<'EOF'
用法:
  ./build.sh

说明:
  自动检查并安装 README 中使用的 apt 常规依赖，
  仅检查 ROS2 Humble 环境是否可用，
  然后统一在仓库根目录下的 build/ 中执行 CMake 配置与编译。
EOF
}

has_apt() {
  command -v apt-get >/dev/null 2>&1 && command -v dpkg-query >/dev/null 2>&1
}

package_installed() {
  dpkg-query -W -f='${Status}' "$1" 2>/dev/null | grep -q '^install ok installed$'
}

run_privileged() {
  if [ "$(id -u)" -eq 0 ]; then
    "$@"
    return
  fi

  if command -v sudo >/dev/null 2>&1; then
    sudo "$@"
    return
  fi

  die "安装缺失依赖需要 root 权限或 sudo。"
}

collect_missing_apt_packages() {
  local package

  MISSING_APT_PACKAGES=()
  for package in "${APT_BUILD_PACKAGES[@]}"; do
    if ! package_installed "${package}"; then
      MISSING_APT_PACKAGES+=("${package}")
    fi
  done
}

install_missing_apt_packages() {
  if [ "${#MISSING_APT_PACKAGES[@]}" -eq 0 ]; then
    log "apt 常规依赖检查通过。"
    return
  fi

  if ! has_apt; then
    die "检测到缺失 apt 常规依赖，但当前系统没有 apt-get/dpkg-query，无法自动安装：${MISSING_APT_PACKAGES[*]}"
  fi

  log "检测到缺失 apt 常规依赖：${MISSING_APT_PACKAGES[*]}"
  log "开始通过 apt 自动安装常规依赖。"

  run_privileged apt-get update
  run_privileged env DEBIAN_FRONTEND=noninteractive apt-get install -y "${MISSING_APT_PACKAGES[@]}"
}

check_ros_environment() {
  local package

  if [ ! -f "${ROS_SETUP}" ]; then
    die "未找到 ${ROS_SETUP}。ROS2 Humble 只做检查、不自动安装，请先手动安装并配置后重试。"
  fi

  if command -v dpkg-query >/dev/null 2>&1; then
    MISSING_ROS_PACKAGES=()
    for package in "${ROS_CHECK_PACKAGES[@]}"; do
      if ! package_installed "${package}"; then
        MISSING_ROS_PACKAGES+=("${package}")
      fi
    done

    if [ "${#MISSING_ROS_PACKAGES[@]}" -ne 0 ]; then
      die "检测到缺失 ROS2 Humble 依赖（不会自动安装）：${MISSING_ROS_PACKAGES[*]}"
    fi
  fi

  # Some ROS setup scripts probe optional env vars before defining them.
  set +u
  # shellcheck disable=SC1090
  source "${ROS_SETUP}"
  set -u
  log "ROS2 Humble 环境检查通过：${ROS_SETUP}"
}

run_cmake_build() {
  local jobs

  mkdir -p "${BUILD_DIR}"
  jobs="$(getconf _NPROCESSORS_ONLN 2>/dev/null || echo 1)"

  log "开始配置 CMake，构建目录：${BUILD_DIR}"
  cmake -S "${SCRIPT_DIR}" -B "${BUILD_DIR}"

  log "开始编译，使用 ${jobs} 个并行任务。"
  cmake --build "${BUILD_DIR}" -j "${jobs}"
}

main() {
  if [ "${1:-}" = "--help" ] || [ "${1:-}" = "-h" ]; then
    show_help
    exit 0
  fi

  if [ "$#" -ne 0 ]; then
    die "暂不支持额外参数，请直接运行 ./build.sh。"
  fi

  cd "${SCRIPT_DIR}"
  collect_missing_apt_packages
  install_missing_apt_packages
  check_ros_environment
  run_cmake_build
  log "构建完成。产物位于 ${BUILD_DIR}"
}

main "$@"
