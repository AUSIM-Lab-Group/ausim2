#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PLUGIN_DIR="${SCRIPT_DIR}/build/bin/mujoco_plugin"

if [ -d "${PLUGIN_DIR}" ]; then
  export MUJOCO_PLUGIN_DIR="${PLUGIN_DIR}"
fi

cd "${SCRIPT_DIR}"
./build/bin/quadrotor "$@"
