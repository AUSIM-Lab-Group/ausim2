# ausim2

基于 MuJoCo 的四旋翼仿真工程，当前主入口是 `quadrotor` C++ 程序。现在的运行时已经拆成：

- MuJoCo 仿真与控制链
- 独立线程运行的 ROS2 bridge
- 配置驱动的机体参数、bindings、话题与 frame

## 目录

```text
ausim2/
├── assets/
├── quadrotor/
│   ├── CMakeLists.txt
│   ├── README.md
│   ├── cfg/
│   │   ├── sim_config.yaml
│   │   └── robot_config.yaml
│   ├── include/
│   └── src/
├── script/
└── third_party/
```

## 构建

依赖：

- CMake >= 3.20
- C++17 编译器
- MuJoCo 3.3.x
- Eigen3
- yaml-cpp
- GLFW
- ROS2 Humble

Ubuntu 常见安装：

```bash
sudo apt install build-essential cmake libeigen3-dev libyaml-cpp-dev libglfw3-dev
source /opt/ros/humble/setup.bash
cmake -S . -B build -DQUADROTOR_ENABLE_ROS2=ON
cmake --build build -j
```

如果 MuJoCo 不在默认路径，可传：

```bash
cmake -S . -B build -Dmujoco_DIR=/path/to/mujoco/build
```

## 运行

默认会自动读取：

- [quadrotor/cfg/sim_config.yaml](quadrotor/cfg/sim_config.yaml)
- [quadrotor/cfg/robot_config.yaml](quadrotor/cfg/robot_config.yaml)

直接运行：

```bash
./build/bin/quadrotor
./build/bin/quadrotor --viewer
./build/bin/quadrotor --headless
```

也支持显式指定：

```bash
./build/bin/quadrotor \
  --sim-config ./quadrotor/cfg/sim_config.yaml \
  --robot-config ./quadrotor/cfg/robot_config.yaml \
  --headless
```

兼容旧的单文件模式：

```bash
./build/bin/quadrotor --config ./path/to/legacy_config.yaml
```

## 配置说明

`sim_config.yaml` 负责全局仿真环境：

- `model.scene_xml`
- `model.track_camera_name`
- `simulation.duration`
- `simulation.dt`
- `simulation.print_interval`
- `viewer.*`
- `ros2.*`

`robot_config.yaml` 负责单台无人机实例：

- `robot.count`
- `identity.*`
- `model.body_name`
- `simulation.control_mode`
- `simulation.example_mode`
- `bindings.*`
- `vehicle.*`
- `controller.*`
- `goal.*`
- `trajectory.*`
- `interfaces.*`
- `frames.*`
- `sensors[]`

`simulation.example_mode` 语义：

- `0`：接收 ROS `cmd_vel`
- `1`：内置简单目标
- `2`：内置圆轨迹 demo

现在 ROS2 bridge 默认开启，不再通过配置开关控制。

## ROS2 使用

默认配置下：

- namespace：`/uav1`
- 订阅：`/uav1/cmd_vel`
- 发布：`/uav1/odom`
- 发布：`/uav1/imu/data`
- 发布：`/clock`
- TF：`uav1/odom -> uav1/base_link`

速度控制示例：

```bash
ros2 topic pub /uav1/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {z: 0.4}}" --once
```

## RMW 说明

在当前实现里，如果环境变量 `RMW_IMPLEMENTATION` 没有显式设置，程序会优先使用 `rmw_cyclonedds_cpp`。

这样做的原因是：在我们的验证环境里，`libmujoco` 与 `rmw_fastrtps_cpp` 同进程启动时会在 ROS2 节点构造阶段触发段错误，而 `rmw_cyclonedds_cpp` 不会出现这个问题。

如果你已经有自己的 RMW 设置，程序会尊重现有环境变量，不会覆盖。

## 详细说明

更完整的架构、线程模型、源码目录和扩展方式见：

[quadrotor/README.md](quadrotor/README.md)
