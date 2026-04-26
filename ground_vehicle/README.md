# Ground Vehicle C++ Runtime

基于 MuJoCo 的地面车（Scout v2 等）仿真工程。与 `quadrotor/` 同构：仿真 + 控制循环在 `scout` 进程，ROS2 bridge 通过本地 socketpair 作为子进程运行，共享 `ausim_common` 里的 runtime / config / ROS 框架。

构建 (`./build.sh`) 与启动 (`./em_run.sh`) 见仓库根 [README](../README.md)；本文件只描述 ground_vehicle 自身的结构、配置与状态机。

产物：`build/bin/scout`、`build/bin/ausim_ros_bridge`。

## 进程模型

```
scout (仿真 + 差速驱动)  <──IPC──>  ausim_ros_bridge (ROS2 I/O)
```

与四旋翼一致地交换两个 DataBoard 槽位：

- `runtime.velocity_command`：bridge → sim，默认来自 `/joy/cmd_vel`
- `runtime.telemetry_snapshot`：sim → bridge，驱动所有 publisher

## 分层职责

| 层 | 路径 | 职责 |
|----|------|------|
| App | `src/main.cpp` | 启动、子进程生命周期、IPC 线程 |
| Sim | `src/sim/` | MuJoCo bindings、状态读取、四轮速度写回、viewer 循环 |
| Control | `src/control/` | 差速驱动控制器（`cmd_vel` → 四轮速度） |
| Config | `src/config/` | YAML 解析到 `GroundVehicleConfig` |
| Runtime / ROS / IPC | `ausim_common/` | 共享的状态机、DataBoard、ROS bridge |

## 配置

### `cfg/sim_config.yaml`

```yaml
robot_config: robot/scout_v2_config.yaml

simulation:
  duration: 0.0
  dt: 0.001
  track_camera_name: "free"

viewer:
  enabled: true
  mjui_enabled: true
  vsync: true

ros2:
  node_name: scout_sim_bridge
  publish_rate_hz: 100.0
  publish_tf: true
  publish_clock: true
```

### `cfg/robot/<name>_config.yaml`

```yaml
identity:
  vehicle_id: scout_v2
  namespace: /scout1
  frame_prefix: scout1

model:
  scene_xml: ../../../assets/scout_v2/scene.xml
  body_name: base_link

bindings:
  freejoint_name: floating_base_joint
  wheel_actuators:
    front_right: velocity_front_right_wheel
    front_left:  velocity_front_left_wheel
    rear_left:   velocity_rear_left_wheel
    rear_right:  velocity_rear_right_wheel
  state:
    gyro_sensor:  body_gyro
    accel_sensor: body_linacc
    quat_sensor:  body_quat

ground_vehicle:
  wheel_radius: 0.165
  track_width:  0.58306
  axle_length:  0.498
  max_linear_speed: 1.5
  max_angular_speed: 2.0
  max_wheel_speed: 20.0
  icr_coefficient: 1.5

interfaces:
  cmd_vel_topic:       /joy/cmd_vel
  joy_cmd_vel_topic:   /joy/cmd_vel
  odom_topic:          odom
  imu_topic:           imu/data
  joy_action_services:
    - { service: /joy/action3, event: reset }
    - { service: /joy/action4, event: estop }
  robot_mode_topic:    teleop/mode

mode_machine: ../teleop/scout_default.yaml
```

### `cfg/teleop/<name>.yaml`

分层状态机定义，与四旋翼同结构（见下节）。默认提供 `scout_default.yaml`。

## ROS2 话题

默认配置（namespace `/scout1`）：

| 方向 | 话题 | 类型 |
|------|------|------|
| 订阅 | `/joy/cmd_vel`            | `geometry_msgs/Twist` |
| 发布 | `/scout1/odom`            | `nav_msgs/Odometry` |
| 发布 | `/scout1/imu/data`        | `sensor_msgs/Imu` |
| 发布 | `/scout1/teleop/mode`     | `std_msgs/String` |
| 发布 | `/clock`                  | `rosgraph_msgs/Clock` |
| 服务 | `/joy/action3`            | `std_srvs/Trigger` |
| 服务 | `/joy/action4`            | `std_srvs/Trigger` |
| TF   | `scout1/odom → scout1/base_link` | — |

## Teleop 状态机

地面车使用同一套状态机引擎，默认配置 `cfg/teleop/scout_default.yaml`：

### 默认状态

| Sub-state      | Top-level       | accepts_motion (cmd_vel) |
|----------------|-----------------|:------------------------:|
| `stopped`      | `MANUAL_READY`  | false |
| `manual_drive` | `MANUAL_ACTIVE` | true  |
| `estop`        | `FAULT`         | false |

地面车不需要 takeoff——`stopped` 是初始状态，`condition: motion_active` 即把状态推到 `manual_drive`，`/joy/cmd_vel` 再被接受。连续 `command_timeout` 秒无 `/joy/cmd_vel` 则自动退回 `stopped`。

### 默认转移

```
stopped      --condition:motion_active---> manual_drive
manual_drive --condition:motion_inactive-> stopped
{any}        --event:estop---------------> estop
```

### 触发途径

| 方式 | 命令 |
|------|------|
| 动作 service | `ros2 service call /joy/action3 std_srvs/srv/Trigger "{}"` |
| 动作 service | `ros2 service call /joy/action4 std_srvs/srv/Trigger "{}"` |
| 速度指令 | `ros2 topic pub /joy/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}, angular: {z: 0.2}}" -r 10` |
| RC 输入 | `third_party/remote_control`，默认把手柄 / 键盘转换到 `/joy/cmd_vel` 与 `/joy/actionN` |

### 自定义

在 `cfg/teleop/` 下新增 YAML，在机型配置里把 `mode_machine:` 指向它即可。状态机引擎与四旋翼 100% 相同——支持 event / condition / timeout 三种触发器与 action 注册表；详见 `quadrotor/README.md` 的 "Teleop 状态机" 章节。
