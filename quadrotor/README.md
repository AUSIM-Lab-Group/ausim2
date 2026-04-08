# Quadrotor C++ Runtime

基于 MuJoCo 的四旋翼仿真工程。代码按 `config / data / converts / controller / runtime / sim / ros` 分层，ROS2 bridge 作为独立子进程运行，通过 IPC 与仿真进程交换数据。

## 构建

依赖：CMake >= 3.20、C++17、MuJoCo 3.3.x、Eigen3、yaml-cpp、GLFW、ROS2 Humble

```bash
source /opt/ros/humble/setup.bash
cmake -S . -B build
cmake --build build -j
```

MuJoCo 不在默认路径时：

```bash
cmake -S . -B build -Dmujoco_DIR=/path/to/mujoco/build
```

产物：`build/bin/quadrotor`、`build/bin/quadrotor_ros_bridge`

## 运行

```bash
./build/bin/quadrotor                    # 默认配置，带 viewer
./build/bin/quadrotor --headless         # 无 GUI

# 显式指定配置
./build/bin/quadrotor \
  --sim-config ./quadrotor/cfg/sim_config.yaml \
  --robot-config ./quadrotor/cfg/robot/crazyfile_config.yaml \
  --headless

# 兼容旧单文件模式
./build/bin/quadrotor --config ./legacy_config.yaml
```

## 进程模型

两个进程通过本地 `socketpair` 通信：

```
quadrotor (仿真 + 控制)  <──IPC──>  quadrotor_ros_bridge (ROS2 I/O)
```

交换的两个数据槽：
- `runtime.velocity_command`：bridge → sim，来自 `/cmd_vel`
- `runtime.telemetry_snapshot`：sim → bridge，驱动所有发布者

## 分层职责

| 层 | 路径 | 职责 |
|----|------|------|
| App/Manager | `src/app/`, `src/manager/` | 启动、子进程生命周期、IPC 线程 |
| Sim | `src/sim/` | MuJoCo bindings、状态读取、执行器写回、viewer 循环 |
| Runtime | `src/runtime/` | `VehicleRuntime` 驱动 SE3 控制器；`GoalProvider` 选择目标来源 |
| Controller | `src/controller/`, `src/control/`, `src/math/` | SE3 控制律、MotorMixer、姿态几何——不依赖 ROS 和 MuJoCo 名称 |
| Data / Converts | `src/data/`, `src/converts/` | 中立消息结构体；内部 ↔ ROS2 ↔ IPC 包格式转换 |
| ROS Bridge | `src/ros/` | ROS2 节点、publisher/subscriber 管理、IPC 收发 |
| Config | `src/config/` | YAML 解析到 `QuadrotorConfig` |

## ROS Bridge 架构

`RosBridgeProcess` 通过两个接口 vector 管理所有 pub/sub：

```cpp
std::vector<std::unique_ptr<ITelemetryPublisher>> publishers_;
std::vector<std::unique_ptr<ICommandSubscriber>>  subscribers_;
```

- `ITelemetryPublisher`（`ros/publisher/i_telemetry_publisher.hpp`）：所有遥测发布者的基类，接收 `TelemetryPacket`，内部完成转换和发布
- `ICommandSubscriber`（`ros/subscriber/i_command_subscriber.hpp`）：所有指令订阅者的基类，回调在构造时注入

`PublishTelemetry()` 只做一件事：

```cpp
for (auto& pub : publishers_) pub->Publish(*packet);
```

`sensors[]` 配置循环驱动额外 publisher 的实例化，新增传感器类型只需在该循环加一个 `else if` 分支。

## 配置

### `cfg/sim_config.yaml`

全局仿真参数和 bridge 参数，以及要加载的机型配置路径：

```yaml
robot_config: robot/crazyfile_config.yaml

simulation:
  duration: 0.0
  dt: 0.001

viewer:
  enabled: true

ros2:
  node_name: sim_bridge
  publish_rate_hz: 100.0
  command_timeout: 0.5
```

### `cfg/robot/<name>_config.yaml`

机型相关参数：

```yaml
identity:
  vehicle_id: cf2
  namespace: /uav1
  frame_prefix: uav1

model:
  scene_xml: ../../../assets/crazyfile/scene.xml
  body_name: cf2

simulation:
  control_mode: 1   # 1=速度控制, 2=位置控制
  example_mode: 0   # 0=ROS cmd_vel, 1=简单目标, 2=圆轨迹
```

## ROS2 话题

默认配置（namespace `/uav1`）：

| 方向 | 话题 | 类型 |
|------|------|------|
| 订阅 | `/uav1/cmd_vel` | `geometry_msgs/Twist` |
| 发布 | `/uav1/odom` | `nav_msgs/Odometry` |
| 发布 | `/uav1/imu/data` | `sensor_msgs/Imu` |
| 发布 | `/clock` | `rosgraph_msgs/Clock` |
| TF | `uav1/odom → uav1/base_link` | — |

发送速度指令：

```bash
ros2 topic pub /uav1/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {z: 0.3}}" --once
```

## 新增机型

复制 `cfg/robot/<name>_config.yaml`，修改 `identity`、`model`、`bindings`、`vehicle`、`controller`，然后将 `sim_config.yaml` 的 `robot_config` 指向新文件。

## 新增传感器

1. 在 MJCF 里添加 sensor / camera / site
2. 在 `cfg/robot/<name>_config.yaml` 的 `sensors[]` 中声明（`enabled: true`、`type`、`topic`、`frame_id`）
3. 在 `src/sim/` 添加读取逻辑（如需要）
4. 在 `src/ros/publisher/data/` 新建 `<type>_data_publisher.hpp/.cpp`，继承 `ITelemetryPublisher`，构造函数接收 `(node, topic, frame_id)`，`Publish(const ipc::TelemetryPacket&)` 内部完成转换和发布
5. 在 `ros2_bridge.cpp` 的 `sensors[]` 循环里为新 `sensor.type` 加一个 `else if` 分支

控制器和混控层无需改动。
