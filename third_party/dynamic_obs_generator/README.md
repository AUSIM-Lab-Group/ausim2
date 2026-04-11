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

在 MuJoCo XML 的 `<worldbody>` 内添加带有 `dynamic_obs_` 前缀名称的 geom：

```xml
<worldbody>
  <!-- 你的其他内容 -->

  <!-- 动态障碍物示例 -->
  <geom name="dynamic_obs_0" type="cylinder" size="0.3 0.5"
        pos="1.0 2.0 1.0" rgba="0.7 0.2 0.2 1" contype="0" conaffinity="0"/>
  <geom name="dynamic_obs_1" type="box" size="0.3 0.3 0.5"
        pos="2.0 1.0 0.5" rgba="0.2 0.7 0.2 1" contype="0" conaffinity="0"/>
  <geom name="dynamic_obs_2" type="sphere" size="0.4"
        pos="0.5 0.5 1.0" rgba="0.2 0.2 0.7 1" contype="0" conaffinity="0"/>
</worldbody>
```

**命名规则**: geom 名称必须以 `dynamic_obs_` 开头，后跟数字（如 `dynamic_obs_0`, `dynamic_obs_1`, `dynamic_obs_2`）

**属性说明**:
- `type`: 形状类型 - `cylinder`, `box`, `sphere`, `cube`
- `size`: 尺寸（MuJoCo half-size 格式）
- `pos`: 初始位置 (x, y, z)
- `rgba`: 颜色（红=0.7 0.2 0.2，绿色=0.2 0.7 0.2，蓝色=0.2 0.2 0.7）
- `contype="0" conaffinity="0"`: 默认无物理碰撞（仅可视化）

### 2. 启用动态障碍物

在 `sim_config.yaml` 中：

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
| `range.x_min/max` | float | -3.0/3.0 | X 范围边界 |
| `range.y_min/max` | float | -3.0/3.0 | Y 范围边界 |
| `range.z_min/max` | float | 0.0/2.0 | Z 范围边界 |

### 运动模式

**2D 模式**: 障碍物在 XY 平面移动，Z 坐标保持固定（用于地面环境）

**3D 模式**: 障碍物在 XYZ 三维空间移动（用于空中环境）

### 静态场景

当 `dynamic: false` 时，仍会生成障碍物，但不会创建运行时运动更新。

## 工作原理

1. **生成**: `generate_scene_obstacles.py` 在启动仿真前基于 `obstacle.yaml` 向场景 XML 注入 `dynamic_obs_*` geoms
2. **扫描**: `DynamicObstacleManager` 在初始化时扫描模型中所有 `dynamic_obs_*` 命名的 geom
3. **记录**: 读取每个 geom 的初始位置、尺寸和运动参数
4. **轨迹播放**: 默认对非物理障碍按深度传感器节拍直接播放预定位置；如果障碍参与碰撞，则退回物理步频更新
5. **碰撞**: 障碍物碰到边界时仅反转对应轴向速度

## 限制

- 障碍物 geom 必须在 XML 场景中预定义（MuJoCo 不支持运行时创建新 geom）
- 障碍物名称必须以 `dynamic_obs_` 开头
- 如果场景中没有匹配的 geom，管理器会输出警告

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
