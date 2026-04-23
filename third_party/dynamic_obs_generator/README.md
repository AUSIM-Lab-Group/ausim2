# Dynamic Obstacle Generator

动态障碍物生成系统 - 在 MuJoCo 仿真环境中动态移动障碍物。

## 架构

```
dynamic_obs_generator/
├── CMakeLists.txt              # 构建配置
├── obstacle_config.hpp/cpp     # 障碍物配置结构和解析
├── dynamic_obstacle_manager.hpp/cpp  # 运行时障碍物管理器
├── obstacle.yaml               # 障碍物运动参数配置
├── generate_scene_obstacles.py # 场景障碍物生成工具
└── README.md                   # 本文档
```

## 使用方法

### 1. 在你的场景 XML 中添加障碍物

在 MuJoCo XML 的 `<worldbody>` 内添加带有 `dynamic_obs_` 前缀名称的障碍物。无碰撞模式可直接使用 geom；需要物理碰撞时，必须使用 mocap body 包裹：

```xml
<worldbody>
  <!-- 你的其他内容 -->

  <!-- 仅可视化/无碰撞 -->
  <geom name="dynamic_obs_0" type="cylinder" size="0.3 0.5"
        pos="1.0 2.0 1.0" rgba="0.7 0.2 0.2 1" contype="0" conaffinity="0"/>

  <!-- 可碰撞：必须使用 mocap body -->
  <body name="dynamic_obs_1" mocap="true" pos="2.0 1.0 0.5">
    <geom name="dynamic_obs_1_geom" type="box" size="0.3 0.3 0.5"
          rgba="0.2 0.7 0.2 1" contype="1" conaffinity="1"/>
  </body>
</worldbody>
```

**命名规则**:
- direct geom 路径: geom 名称必须以 `dynamic_obs_` 开头
- mocap 路径: body 名称必须以 `dynamic_obs_` 开头，子 geom 推荐命名为 `<body_name>_geom`

**属性说明**:
- `type`: 形状类型 - `cylinder`, `box`, `sphere`, `cube`
- `size`: 尺寸（MuJoCo half-size 格式）
- `pos`: 初始位置 (x, y, z)
- `rgba`: 颜色（红=0.7 0.2 0.2，绿色=0.2 0.7 0.2，蓝色=0.2 0.2 0.7）
- `contype="0" conaffinity="0"`: 默认无物理碰撞（仅可视化）
- `mocap="true"`: 可碰撞障碍物的推荐驱动方式；运行时通过 `data->mocap_pos` 更新，不直接 teleport geom

### 2. 启用动态障碍物

在模型 registry（推荐）或合并配置中：

```yaml
dynamic_obstacle:
  enabled: true
  config_path: "../../third_party/dynamic_obs_generator/obstacle.yaml"
```

### 3. 配置运动参数

编辑 `obstacle.yaml`：

```yaml
# 随机种子
random_seed: 42

# 是否启用运行时运动
dynamic: true

# 是否输出逐障碍初始化日志和完整调试信息
debug: false

# 运动模式: "2d" 或 "3d"
mode: "2d"

# 统一 box / cube 边长
box_size: 0.1

# 生成范围
range:
  x_min: -3.0
  x_max: 3.0
  y_min: -3.0
  y_max: 3.0
  z_min: 0.0
  z_max: 2.0

# 障碍物数量（会自动扫描场景中的 dynamic_obs_* geoms）
obstacle_count: 10

# 运动速度
min_speed: 0.0    # 最小速度 (m/s)
max_speed: 0.5   # 最大速度 (m/s)

# 可选：向 ROS bridge 发布所有 dynamic_obs_* 当前包围盒
# 注意：publish.enabled 只控制 ROS 话题发布，不控制障碍物生成或运行时运动。
publish:
  enabled: false
  topic: /dyn_obstacle
  frame_id: world
  rate_hz: 20.0
```

### 4. 编译运行

```bash
cmake --build build -j
./build/bin/quadrotor --sim-config ./quadrotor/cfg/sim_config.yaml
```

## 配置参数详解

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `random_seed` | int | 42 | 随机种子，用于重现 |
| `dynamic` | bool | true | 是否启用运行时障碍运动 |
| `debug` | bool | false | 是否打印逐障碍初始化日志和完整调试信息 |
| `mode` | string | "2d" | "2d" 或 "3d" 运动模式 |
| `box_size` | float | 0.5 | box / cube 共用边长 |
| `obstacle_count` | int | 10 | 最多扫描的障碍物数量 |
| `min_speed` | float | 0.0 | 最小速度 |
| `max_speed` | float | 0.5 | 最大速度 |
| `collision_enabled` | bool | false | 生成可碰撞障碍；启用时 Python 侧会生成 mocap body 包裹 |
| `publish.enabled` | bool | false | 是否发布 `/dyn_obstacle` 包围盒数组；只控制 ROS 发布，不控制障碍物生成或运动 |
| `publish.topic` | string | `/dyn_obstacle` | 障碍物数组话题名 |
| `publish.frame_id` | string | `world` | 发布使用的默认坐标系 |
| `publish.rate_hz` | float | 20.0 | 障碍物数组发布频率 |
| `range.x_min/max` | float | -3.0/3.0 | X 范围边界 |
| `range.y_min/max` | float | -3.0/3.0 | Y 范围边界 |
| `range.z_min/max` | float | 0.0/2.0 | Z 范围边界 |

### 运动模式

**2D 模式**: 障碍物在 XY 平面移动，Z 坐标保持固定（用于地面环境）

**3D 模式**: 障碍物在 XYZ 三维空间移动（用于空中环境）

### 静态场景

当 `dynamic: false` 时，仍会生成障碍物，但不会创建运行时运动更新。

## 话题发布

### 开关语义

- `dynamic_obstacle.enabled` 决定是否初始化运行时动态障碍管理器
- `publish.enabled` 只决定是否向 ROS bridge 发布 `/dyn_obstacle`
- 两者不是同一个开关，不应混用

这意味着：

- `dynamic_obstacle.enabled=true` 且 `publish.enabled=false`：障碍物仍会生成并运动，但不发布 ROS 话题
- `dynamic_obstacle.enabled=true` 且 `publish.enabled=true`：障碍物生成、运动，并发布 `/dyn_obstacle`
- `dynamic_obstacle.enabled=false`：不会初始化运行时动态障碍管理器，`publish.enabled` 也不会产生实际输出

### 启动条件

要实际看到 `/dyn_obstacle` 消息，至少需要同时满足：

1. `dynamic_obstacle.enabled=true`
2. 启动时加载的场景里确实存在 `dynamic_obs_*`
3. `publish.enabled=true`
4. 观测 ROS 话题的 shell 已同时执行：

```bash
source /opt/ros/humble/setup.bash
source build/ros_ws/install/setup.bash
```

如果通过 `./em_run.sh` 启动，并且 registry 中模型开启了 `dynamic_obstacle.enabled`，脚本会先调用 `generate_scene_obstacles.py` 生成带 `dynamic_obs_*` 的场景，再启动仿真。  
如果直接运行 `build/bin/scout` 或 `build/bin/quadrotor`，则需要你自己传入已经生成好的场景。

### 运行模式一致性

当前实现下，无论是否启用 viewer，动态障碍工作流保持一致：

- `headless` 路径：每次物理步进后都会更新障碍物状态，并在到达 `publish.rate_hz` 时产出 snapshot
- `viewer` 路径：每次物理步进后走同一套 snapshot 发布逻辑；暂停态也会发布当前障碍物快照
- 因此 `/dyn_obstacle` 的话题语义在两种模式下保持一致，差别只在是否显示 GUI，不在数据链路

### 观测示例

```bash
source /opt/ros/humble/setup.bash
source build/ros_ws/install/setup.bash

ros2 topic echo --once /dyn_obstacle
ros2 topic hz /dyn_obstacle
```

发布消息类型为 `ausim_msg/msg/BoundingBox3DArray`。

## 工作原理

1. **生成**: `generate_scene_obstacles.py` 在启动仿真前基于 `obstacle.yaml` 注入障碍物
2. **分支**:
   - `collision_enabled: false` 时生成 direct geom，运行时直接写 `model->geom_pos`
   - `collision_enabled: true` 时生成 `mocap="true"` body，运行时写 `data->mocap_pos`
3. **扫描**: `DynamicObstacleManager` 初始化时优先识别 `dynamic_obs_*` mocap body，其次回退到 direct geom
4. **轨迹播放**:
   - mocap 路径保持接触求解兼容，通常可继续按轨迹播放器节拍更新
   - direct geom 路径适合纯视觉障碍；如果用户手写了可碰撞 direct geom，管理器会警告并继续工作
5. **运行模式一致**:
   - `headless` 步进后会走 snapshot 发布逻辑
   - `viewer` 步进后也会走同一逻辑；暂停时发布当前快照
6. **桥接发布**: 若 `publish.enabled=true`，sim 进程会按 `publish.rate_hz` 采样当前障碍物位置/尺寸，经 IPC 发给 ROS bridge，发布成 `ausim_msg/msg/BoundingBox3DArray`
7. **碰撞**: 障碍物碰到边界时仅反转对应轴向速度

## 数据流示意

```mermaid
flowchart TD
    A[registry / sim config<br/>dynamic_obstacle.enabled] --> B[em_run.sh 或手工生成场景]
    B --> C[generate_scene_obstacles.py]
    C --> D[scene.dynamic_obstacles.xml<br/>包含 dynamic_obs_*]
    D --> E[Scout / Quadrotor 仿真加载场景]
    F[obstacle.yaml<br/>dynamic / collision / publish 配置] --> E
    E --> G[DynamicObstacleManager<br/>扫描并驱动 dynamic_obs_*]
    G --> H[每次物理步进更新位姿]
    H --> I[到达 publish.rate_hz<br/>构造 snapshot]
    I --> J[DataBoard / IPC]
    J --> K[ROS bridge]
    K --> L[/dyn_obstacle<br/>ausim_msg/msg/BoundingBox3DArray]
```

## 常见排查点

- 只有 `/dyn_obstacle` topic 没有消息：先确认当前版本是否包含 viewer 路径发布修复，再确认 `publish.enabled=true`
- `ros2 topic echo /dyn_obstacle` 无法解析类型：通常是观测 shell 没有 `source build/ros_ws/install/setup.bash`
- 障碍物会动但不发话题：通常是 `dynamic_obstacle.enabled=true` 但 `publish.enabled=false`
- `publish.enabled=true` 仍无输出：检查实际加载的 scene 是否真的包含 `dynamic_obs_*`

## 限制

- 障碍物仍需在启动前写入 XML（MuJoCo 不支持运行时创建新 geom）
- 需要物理碰撞的障碍物必须用 mocap body 包裹；对可碰撞裸 geom 的 teleport 只保留为兼容路径，并会输出 warning
- 如果场景中没有匹配的 `dynamic_obs_*` body/geom，管理器会输出警告

## Python 场景生成工具

可以使用 `generate_scene_obstacles.py` 快速生成带障碍物的场景：

```bash
python3 third_party/dynamic_obs_generator/generate_scene_obstacles.py \
    --sim-config quadrotor/cfg/sim_config.yaml \
    --print-output-path
```

支持的形状:
- `cylinder`: 圆柱体（2D 模式）
- `box`: 长方体（2D 模式）或立方体（3D 模式）
- `sphere`: 球体（3D 模式）
