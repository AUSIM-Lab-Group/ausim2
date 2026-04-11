#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PLUGIN_DIR="${SCRIPT_DIR}/build/bin/mujoco_plugin"
GENERATOR="${SCRIPT_DIR}/third_party/dynamic_obs_generator/generate_scene_obstacles.py"

if [ -d "${PLUGIN_DIR}" ]; then
  export MUJOCO_PLUGIN_DIR="${PLUGIN_DIR}"
fi

MERGED_CONFIG=""
SIM_CONFIG=""
ROBOT_CONFIG=""
SHOW_HELP=0

ARGS=("$@")
for ((i=0; i<${#ARGS[@]}; ++i)); do
  arg="${ARGS[$i]}"
  case "${arg}" in
    --help|-h)
      SHOW_HELP=1
      ;;
    --config)
      if [ $((i + 1)) -lt ${#ARGS[@]} ]; then
        MERGED_CONFIG="${ARGS[$((i + 1))]}"
        ((++i))
      fi
      ;;
    --sim-config)
      if [ $((i + 1)) -lt ${#ARGS[@]} ]; then
        SIM_CONFIG="${ARGS[$((i + 1))]}"
        ((++i))
      fi
      ;;
    --robot-config)
      if [ $((i + 1)) -lt ${#ARGS[@]} ]; then
        ROBOT_CONFIG="${ARGS[$((i + 1))]}"
        ((++i))
      fi
      ;;
    --viewer|--headless)
      ;;
    -*)
      ;;
    *)
      if [ -z "${MERGED_CONFIG}" ] && [ -z "${SIM_CONFIG}" ]; then
        MERGED_CONFIG="${arg}"
      fi
      ;;
  esac
done

cd "${SCRIPT_DIR}"

if [ "${SHOW_HELP}" -eq 0 ] && [ -f "${GENERATOR}" ]; then
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

exec ./build/bin/quadrotor "$@"
