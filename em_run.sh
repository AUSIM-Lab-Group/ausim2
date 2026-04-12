#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PLUGIN_DIR="${SCRIPT_DIR}/build/bin/mujoco_plugin"
BUNDLED_PLUGIN_DIR="${SCRIPT_DIR}/third_party/mujoco-3.6.0/bin/mujoco_plugin"
GENERATOR="${SCRIPT_DIR}/third_party/dynamic_obs_generator/generate_scene_obstacles.py"
MODEL_TARGET_CONFIG="${SCRIPT_DIR}/.em_run_target"

if [ -d "${BUNDLED_PLUGIN_DIR}" ] && [ -d "${PLUGIN_DIR}" ]; then
  export MUJOCO_PLUGIN_DIR="${BUNDLED_PLUGIN_DIR}:${PLUGIN_DIR}"
elif [ -d "${BUNDLED_PLUGIN_DIR}" ]; then
  export MUJOCO_PLUGIN_DIR="${BUNDLED_PLUGIN_DIR}"
elif [ -d "${PLUGIN_DIR}" ]; then
  export MUJOCO_PLUGIN_DIR="${PLUGIN_DIR}"
fi

MERGED_CONFIG=""
SIM_CONFIG=""
ROBOT_CONFIG=""
SHOW_HELP=0
FORCE_SELECT=0
MODEL_TARGET=""
TARGET_FAMILY=""
EXECUTABLE=""
PASSTHROUGH_ARGS=()

normalize_model_target() {
  case "$1" in
    crazyfile)
      printf "crazyfile\n"
      return 0
      ;;
    scout_v2)
      printf "scout_v2\n"
      return 0
      ;;
    *)
      return 1
      ;;
  esac
}

choose_model_target() {
  local choice=""
  local selected=""

  echo "请选择要启动的仿真模型："
  echo "  1) crazyfile (quadrotor)"
  echo "  2) scout_v2 (ground_vehicle)"

  while true; do
    read -r -p "输入编号或名称: " choice
    case "${choice}" in
      ""|1|crazyfile)
        selected="crazyfile"
        break
        ;;
      2|scout_v2)
        selected="scout_v2"
        break
        ;;
      *)
        echo "无效选择：${choice}。请输入 1/2、crazyfile 或 scout_v2。"
        ;;
    esac
  done

  printf "%s\n" "${selected}" > "${MODEL_TARGET_CONFIG}"
  echo "已保存默认仿真模型：${selected}（之后用 ./em_run.sh -S 可重新选择）"
  MODEL_TARGET="${selected}"
}

load_model_target() {
  local stored_target=""

  if [ "${FORCE_SELECT}" -eq 1 ]; then
    choose_model_target
    return
  fi

  if [ -f "${MODEL_TARGET_CONFIG}" ]; then
    stored_target="$(tr -d '[:space:]' < "${MODEL_TARGET_CONFIG}")"
    if MODEL_TARGET="$(normalize_model_target "${stored_target}")"; then
      echo "使用默认仿真模型：${MODEL_TARGET}（之后用 ./em_run.sh -S 可重新选择）"
      return
    fi
    echo "已保存的仿真模型无效：${stored_target}，将重新选择。"
  fi

  if [ -t 0 ]; then
    choose_model_target
  else
    MODEL_TARGET="crazyfile"
    printf "%s\n" "${MODEL_TARGET}" > "${MODEL_TARGET_CONFIG}"
    echo "未检测到交互式终端，默认选择 crazyfile。之后用 ./em_run.sh -S 可重新选择。"
  fi
}

resolve_runtime_profile() {
  case "${MODEL_TARGET}" in
    crazyfile)
      TARGET_FAMILY="quadrotor"
      EXECUTABLE="./build/bin/quadrotor"
      if [ -z "${MERGED_CONFIG}" ] && [ -z "${SIM_CONFIG}" ]; then
        PASSTHROUGH_ARGS+=(--sim-config "${SCRIPT_DIR}/quadrotor/cfg/sim_config.yaml")
        SIM_CONFIG="${SCRIPT_DIR}/quadrotor/cfg/sim_config.yaml"
      fi
      ;;
    scout_v2)
      TARGET_FAMILY="ground_vehicle"
      EXECUTABLE="./build/bin/scout"
      if [ -z "${MERGED_CONFIG}" ] && [ -z "${SIM_CONFIG}" ]; then
        PASSTHROUGH_ARGS+=(--sim-config "${SCRIPT_DIR}/ground_vehicle/cfg/sim_config.yaml")
        SIM_CONFIG="${SCRIPT_DIR}/ground_vehicle/cfg/sim_config.yaml"
      fi
      ;;
    *)
      echo "未知仿真模型：${MODEL_TARGET}" >&2
      exit 1
      ;;
  esac

  if [ ! -x "${EXECUTABLE}" ]; then
    echo "目标可执行文件不存在或不可执行：${EXECUTABLE}" >&2
    echo "请先构建对应目标，例如：cmake --build build --target ${EXECUTABLE##*/}" >&2
    exit 1
  fi
}

ARGS=("$@")
for ((i=0; i<${#ARGS[@]}; ++i)); do
  arg="${ARGS[$i]}"
  case "${arg}" in
    -S)
      FORCE_SELECT=1
      ;;
    --help|-h)
      SHOW_HELP=1
      PASSTHROUGH_ARGS+=("${arg}")
      ;;
    --config)
      PASSTHROUGH_ARGS+=("${arg}")
      if [ $((i + 1)) -lt ${#ARGS[@]} ]; then
        MERGED_CONFIG="${ARGS[$((i + 1))]}"
        PASSTHROUGH_ARGS+=("${MERGED_CONFIG}")
        ((++i))
      fi
      ;;
    --sim-config)
      PASSTHROUGH_ARGS+=("${arg}")
      if [ $((i + 1)) -lt ${#ARGS[@]} ]; then
        SIM_CONFIG="${ARGS[$((i + 1))]}"
        PASSTHROUGH_ARGS+=("${SIM_CONFIG}")
        ((++i))
      fi
      ;;
    --robot-config)
      PASSTHROUGH_ARGS+=("${arg}")
      if [ $((i + 1)) -lt ${#ARGS[@]} ]; then
        ROBOT_CONFIG="${ARGS[$((i + 1))]}"
        PASSTHROUGH_ARGS+=("${ROBOT_CONFIG}")
        ((++i))
      fi
      ;;
    --viewer|--headless)
      PASSTHROUGH_ARGS+=("${arg}")
      ;;
    -*)
      PASSTHROUGH_ARGS+=("${arg}")
      ;;
    *)
      if [ -z "${MERGED_CONFIG}" ] && [ -z "${SIM_CONFIG}" ]; then
        MERGED_CONFIG="${arg}"
      fi
      PASSTHROUGH_ARGS+=("${arg}")
      ;;
  esac
done

cd "${SCRIPT_DIR}"

load_model_target
resolve_runtime_profile

if [ "${SHOW_HELP}" -eq 0 ] && [ "${TARGET_FAMILY}" = "quadrotor" ] && [ -f "${GENERATOR}" ]; then
  GENERATOR_ARGS=(--print-output-path)
  if [ -n "${MERGED_CONFIG}" ]; then
    GENERATOR_ARGS+=(--config "${MERGED_CONFIG}")
  else
    if [ -n "${SIM_CONFIG}" ]; then
      GENERATOR_ARGS+=(--sim-config "${SIM_CONFIG}")
    fi
    if [ -n "${ROBOT_CONFIG}" ]; then
      GENERATOR_ARGS+=(--robot-config "${ROBOT_CONFIG}")
    fi
  fi

  GENERATED_SCENE_XML="$(python3 "${GENERATOR}" "${GENERATOR_ARGS[@]}")"
  export AUSIM_SCENE_XML_OVERRIDE="${GENERATED_SCENE_XML}"
fi

exec "${EXECUTABLE}" "${PASSTHROUGH_ARGS[@]}"
