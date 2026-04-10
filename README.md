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
- MuJoCo 3.6.x
- Eigen3
- yaml-cpp
- GLFW
- ROS2 Humble

MuJoCo 推荐按源码编译安装：

```bash
cmake -S /home/x/mujoco/mujoco-3.6.0 -B /home/x/mujoco/mujoco-3.6.0/build \
  -DCMAKE_INSTALL_PREFIX=/opt/mujoco
cmake --build /home/x/mujoco/mujoco-3.6.0/build -j
sudo cmake --install /home/x/mujoco/mujoco-3.6.0/build
```

项目构建：

```bash
sudo apt install build-essential cmake libeigen3-dev libyaml-cpp-dev libglfw3-dev
source /opt/ros/humble/setup.bash
cmake -S . -B build -DMUJOCO_SOURCE_DIR=/home/x/mujoco/mujoco-3.6.0
cmake --build build -j
```

现在的搜索逻辑尽量只需要 `MUJOCO_SOURCE_DIR`：

- 先读 `${MUJOCO_SOURCE_DIR}/build/CMakeCache.txt`
- 从里面提取 `CMAKE_INSTALL_PREFIX`
- 自动尝试 `${install_prefix}/lib/cmake/mujoco`
- 如果没装 package，再回退到 `${MUJOCO_SOURCE_DIR}/include` 和 `${MUJOCO_SOURCE_DIR}/build/lib/libmujoco.so`

只有高级场景才需要手工指定：

```bash
cmake -S . -B build -Dmujoco_DIR=/path/to/mujoco/lib/cmake/mujoco
```

主工程会自动编译 `third_party/mujoco_ray_caster`，并把插件库输出到：

- `build/bin/mujoco_plugin/libsensor_raycaster.so`

可选 smoke test：

```bash
cmake -S . -B build \
  -DMUJOCO_SOURCE_DIR=/path/to/mujoco-source \
  -DMUJOCO_RAY_CASTER_BUILD_SMOKETEST=ON
cmake --build build -j
./build/bin/mujoco_ray_caster_smoketest
```

### third_party/mujoco_ray_caster（已并入主工程）

当前仓库将 `third_party/mujoco_ray_caster` 作为主仿真链路的一部分直接构建，不再保留独立 demo 文档结构。

- 插件目标：`sensor_raycaster`
- 输出路径：`build/bin/mujoco_plugin/libsensor_raycaster.so`
- 主要插件类型：
  - `mujoco.sensor.ray_caster`
  - `mujoco.sensor.ray_caster_camera`
  - `mujoco.sensor.ray_caster_lidar`

噪声与立体模型相关的推导说明保留在：

- `third_party/mujoco_ray_caster/compute.md`
- `third_party/mujoco_ray_caster/compute.zh-CN.md`

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

运行时会优先读取 `MUJOCO_PLUGIN_DIR`；如果未设置，则自动尝试加载可执行文件旁边的 `mujoco_plugin/` 目录，也就是默认的 `build/bin/mujoco_plugin/`。

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
- `simulation.track_camera_name`
- `simulation.duration`
- `simulation.dt`
- `simulation.print_interval`
- `simulation.control_mode`
- `simulation.example_mode`
- `goal.*`
- `trajectory.*`
- `viewer.*`
- `ros2.*`

`robot_config.yaml` 负责单台无人机实例：

- `robot.count`
- `identity.*`
- `model.body_name`
- `model.aircraft_forward_axis`
- `bindings.*`
- `vehicle.*`
- `controller.*`
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

## 详细说明

更完整的架构、线程模型、源码目录和扩展方式见：

[quadrotor/README.md](quadrotor/README.md)
