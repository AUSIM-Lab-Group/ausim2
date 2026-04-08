# Quadrotor C++ Runtime

这个目录是当前四旋翼仿真的主工程。代码按 `config / data / converts / runtime / sim / ros` 分层，目标是让机型切换、bindings 适配和传感器扩展都尽量配置化，同时把消息格式、格式转换和发布逻辑拆开。

## 目录结构

```text
quadrotor/
├── CMakeLists.txt
├── README.md
├── cfg/
│   ├── robot_config.yaml
│   └── sim_config.yaml
└── src/
    ├── converts/
    ├── config/
    ├── control/
    ├── controller/
    ├── data/
    ├── math/
    ├── ros/
    ├── runtime/
    ├── sim/
    └── main.cpp
```

## 分层职责

### `config/`

负责配置结构体和 YAML 解析。

- [quadrotor_config.hpp](src/config/quadrotor_config.hpp)
- [quadrotor_config.cpp](src/config/quadrotor_config.cpp)

加载方式支持：

- 两文件模式：`sim_config.yaml + robot_config.yaml`
- 旧版单文件模式：`--config legacy.yaml`

解析策略是“顺序覆盖”：

1. 先加载 `sim_config.yaml`
2. 再加载 `robot_config.yaml`

这样既能把全局仿真和单机配置拆开，也保留局部覆盖能力。

### `data/` + `converts/`

消息格式层和格式转换层：

- `data/`：保存中立、可移植的消息格式定义
- `converts/data/`：负责内部状态、这些中立格式以及 ROS 消息之间的转换

当前已经拆出的消息包括：

- `cmd_vel`
- `odom`
- `imu`
- `tf`
- `/clock`

这样做的目的，是让每个消息包都有单独的格式文件，便于查看字段、后续迁移到别的中间件，或者替换成非 ROS 的出口。

### `controller/` + `control/` + `math/`

纯控制核心：

- 状态表示
- SE(3) 控制律
- 电机混控
- 姿态几何工具

这一层不依赖 ROS2，也不关心 MuJoCo 名称绑定。

### `runtime/`

把“当前状态”变成“当前控制输出”。

- `CommandMailbox`
- `TelemetryCache`
- `GoalProvider`
- `VehicleRuntime`

当前控制来源：

- `example_mode=0`：`CommandGoalProvider`
- `example_mode=1/2`：`DemoGoalProvider`

当 `example_mode=0` 且暂时没有新鲜 `cmd_vel` 时，运行时不会回退到 demo，而是保持当前位置悬停。

### `sim/`

MuJoCo 适配层：

- 模型名称到 id / adr / dim 的绑定
- 状态读取
- 执行器写回
- viewer / headless 运行循环

这一层不直接处理 ROS 话题。

### `ros/`

ROS2 bridge：

- bridge 启停和 executor 生命周期管理
- `ros/publisher/data/` 下的 `odom` / `imu` / `tf` / `/clock` 发布
- `ros/subscriber/data/` 下的 `cmd_vel` 订阅
- 独立线程运行 `SingleThreadedExecutor`

当前 `sensors[]` 里的相机和激光雷达还是预留扩展入口，但基础消息已经拆成独立 publisher/subscriber helper，`ros2_bridge.cpp` 不再直接手写每个字段。

## 线程模型

当前主流程是两条线程：

1. 仿真线程
2. ROS2 bridge 线程

线程之间只通过两个轻量对象交互：

- `CommandMailbox`
- `TelemetryCache`

这样 MuJoCo callback 不需要直接触碰 ROS2 executor。

## 配置拆分

### `cfg/sim_config.yaml`

放全局仿真环境和 bridge 基础参数：

```yaml
model:
  scene_xml: ../../assets/crazyfile/scene.xml
  track_camera_name: track

simulation:
  duration: 0.0
  dt: 0.001
  print_interval: 0.0

viewer:
  enabled: true

ros2:
  node_name: sim_bridge
  publish_rate_hz: 100.0
  command_timeout: 0.5
```

### `cfg/robot_config.yaml`

放单机实例参数：

```yaml
robot:
  count: 1

identity:
  vehicle_id: cf2
  namespace: /uav1
  frame_prefix: uav1

model:
  body_name: cf2

simulation:
  control_mode: 1
  example_mode: 0
```

关键字段：

- `robot.count`
  - 当前必须是 `1`
  - 多机实例化还没实现
- `simulation.control_mode`
  - `1`：速度控制
  - `2`：位置控制
- `simulation.example_mode`
  - `0`：ROS `cmd_vel`
  - `1`：简单目标
  - `2`：圆轨迹

## 运行方式

### 默认配置

```bash
./build/bin/quadrotor
./build/bin/quadrotor --viewer
./build/bin/quadrotor --headless
```

### 显式指定分离配置

```bash
./build/bin/quadrotor \
  --sim-config ./quadrotor/cfg/sim_config.yaml \
  --robot-config ./quadrotor/cfg/robot_config.yaml \
  --headless
```

### 兼容旧配置

```bash
./build/bin/quadrotor --config ./legacy_config.yaml
```

## ROS2 话题

默认配置下：

- 订阅：`/uav1/cmd_vel`
- 发布：`/uav1/odom`
- 发布：`/uav1/imu/data`
- 发布：`/clock`
- TF：`uav1/odom -> uav1/base_link`

发布控制指令：

```bash
ros2 topic pub /uav1/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {z: 0.3}}" --once
```

## RMW 选择

ROS2 bridge 现在默认开启，不再有 `ros2.enabled` 配置开关。

如果环境变量 `RMW_IMPLEMENTATION` 没有设置，程序会自动优先选择 `rmw_cyclonedds_cpp`。原因是我们确认过：

- `libmujoco + rmw_fastrtps_cpp` 同进程启动时，ROS2 节点构造阶段会触发段错误
- `libmujoco + rmw_cyclonedds_cpp` 不会触发这个段错误

如果你需要强制指定 RMW，可以在运行前显式导出：

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
./build/bin/quadrotor --headless
```

## 如何新增机型

推荐只复制并修改 `robot_config.yaml`：

1. 修改 `identity.namespace` 和 `identity.frame_prefix`
2. 修改 `model.body_name`
3. 修改 `bindings`
4. 修改 `vehicle`
5. 修改 `interfaces` 和 `frames`

如果场景 XML 也不同，再改 `sim_config.yaml` 里的 `model.scene_xml`。

## 如何新增传感器

建议顺序：

1. 在 MJCF 里加 sensor / camera / site
2. 在 `robot_config.yaml` 的 `sensors[]` 里声明
3. 在 `sim/` 增加读取逻辑
4. 在 `ros/` 增加对应 publisher

控制器和混控层通常不需要改。
