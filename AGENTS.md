# AGENTS.md

## Scope

These instructions apply to the entire repository unless a deeper `AGENTS.md`
overrides them.

## Repository Overview

- `ausim_common/`: shared runtime, config parsing, IPC, ROS2 bridge helpers,
  state machine, and common publishers/subscribers.
- `quadrotor/`: quadrotor simulation, control, app/runtime flow, and robot
  configs.
- `ground_vehicle/`: Scout-style ground vehicle simulation, control, and robot
  configs.
- `assets/`: MJCF scenes, meshes, robot assets, and generated scene variants.
- `third_party/`: vendored dependencies and external components.
- `build/`: generated build outputs. Do not treat this as source.

## Working Rules

- Prefer minimal, scoped changes that match the existing CMake, ROS2, MuJoCo,
  and YAML patterns already used in the repo.
- Preserve the current layering instead of introducing cross-layer shortcuts.
  In `quadrotor/`, keep logic separated across `config`, `data`, `converts`,
  `controller`, `runtime`, `sim`, and `ros` where applicable.
- Keep shared ROS/runtime or IPC behavior in `ausim_common/` when the change
  affects both quadrotor and ground vehicle flows.
- Do not modify `third_party/` by default. Only touch it when the task
  explicitly targets vendored code, message packages, or external tooling.
- Do not commit `build/` artifacts.
- Avoid editing generated scene outputs such as
  `assets/*/scene.dynamic_obstacles.xml` unless the task is specifically about
  dynamic obstacle generation.

## Environment And Prerequisites

- Expected environment is Ubuntu 22.04 style tooling with `apt`.
- ROS2 Humble is required at `/opt/ros/humble/setup.bash`.
- `colcon` must be available for the ROS overlay build.
- `./build.sh` may install missing apt packages with `sudo`, but it does not
  install ROS2 for you. It only validates that the ROS2 Humble environment is
  already present.
- When running ROS2 commands in a fresh shell, source:

```bash
source /opt/ros/humble/setup.bash
```

## Build And Run

- Default build command: `./build.sh`
- Preferred end-to-end run commands:

```bash
./em_run.sh --headless
./em_run.sh --viewer
```

- Default sim configs:
  - `quadrotor/cfg/sim_config.yaml`
  - `ground_vehicle/cfg/sim_config.yaml`
- Primary runtime binaries produced by the build:
  - `build/bin/quadrotor`
  - `build/bin/scout`
  - `build/bin/quadrotor_ros_bridge`
  - `build/bin/ausim_ros_bridge`

## Formatting

- For C++ edits, run:

```bash
./format.sh
```

- `format.sh` expects `clang-format` v15 and skips `build/` and `third_party/`.

## Validation

- After changes, prefer the smallest matching smoke test before broader
  end-to-end validation.
- If you changed shared build logic or config loading, run `./build.sh`.
- If you changed quadrotor runtime or app flow, prefer:

```bash
timeout 2s ./build/bin/quadrotor --sim-config quadrotor/cfg/sim_config.yaml --headless
```

- If you changed ground vehicle runtime or app flow, prefer:

```bash
timeout 2s ./build/bin/scout --sim-config ground_vehicle/cfg/sim_config.yaml --headless
```

- If you changed the MuJoCo ray-caster plugin path, use the repo's plugin smoke
  test when available.
- If the task is about full stack behavior, ROS topics, or launch integration,
  follow the targeted smoke test with `./em_run.sh --headless`.

## Change Boundaries

- Prefer not to widen scope into unrelated refactors while touching simulator
  logic.
- Preserve public topic names, config keys, and launch behavior unless the task
  explicitly requires changing them.
- Keep generated or vendored content out of patches unless it is the direct
  target of the request.
