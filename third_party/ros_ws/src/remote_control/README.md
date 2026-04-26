# Remote Control

仓内 ROS2 遥控适配包：把手柄和键盘输入转换为连续速度命令，以及一组
`std_srvs/srv/Trigger` 离散动作调用。节点本身不理解起飞、降落、重置等
机器人语义，只负责把输入映射到配置文件声明的 topic 和 service。

## 包结构

```text
third_party/ros_ws/src/remote_control/
├── CMakeLists.txt
├── package.xml
├── launch/
│   ├── remote_control.launch.py
│   └── remote_control_gui.launch.py
├── config/
│   ├── xbox_like.yaml
│   └── xbox_like.md
└── src/
    ├── gui_control_model.cpp
    ├── gui_control_model.hpp
    ├── gui_runtime_config.cpp
    ├── gui_runtime_config.hpp
    ├── main.cpp
    ├── remote_control_gui_main.cpp
    ├── remote_control_node.cpp
    └── remote_control_node.hpp
```

- `launch/remote_control.launch.py`：终端遥控入口；启动 `remote_control_node`，并在检测到 `/dev/input/js*` 时自动启动 `joy_node`。
- `launch/remote_control_gui.launch.py`：Qt5 图形遥控入口；启动 `remote_control_gui_node` 可执行文件，节点名沿用 `remote_control_node` 以复用默认 YAML，默认同时启动 `joy_node`，并把当前 `config` 路径注入为 `gui.config_path` 供 GUI 保存。
- `config/xbox_like.yaml`：默认参数，包含 topic、轴映射、速度缩放、GUI 语言、动作服务和键盘设置。
- `config/xbox_like.md`：默认 YAML 的逐项说明。
- `src/remote_control_node.*`：终端节点；订阅 `sensor_msgs/msg/Joy`，读取前台终端 stdin 键盘输入，发布 `Twist` 并触发 `Trigger` service。
- `src/remote_control_gui_main.cpp`：Qt5 控制窗口；接收窗口焦点键盘事件、显示输入来源和手柄状态，复用 `/joy` 手柄输入，并支持在线编辑/保存速度与 action 映射。

## 运行流程

```text
手柄 /dev/input/jsN
  -> joy_node
  -> /joy (sensor_msgs/Joy)
  -> remote_control_gui_node / remote_control_node
  -> topics.cmd_vel (geometry_msgs/Twist)

前台终端 stdin
  -> remote_control_node
  -> topics.cmd_vel (geometry_msgs/Twist)

GUI 窗口键盘事件
  -> remote_control_gui_node
  -> topics.cmd_vel (geometry_msgs/Twist)

手柄组合键或键盘动作键
  -> remote_control_gui_node / remote_control_node
  -> /joy/actionN (std_srvs/Trigger)
  -> 仿真桥 / 状态机
```

当前默认配置订阅 `/joy`，并把 `topics.cmd_vel` 直接写成绝对路径
`/joy/cmd_vel`，与仓内仿真桥订阅的 `joy_cmd_vel_topic` 默认值对齐，
开箱即用。如果接入的是其他订阅端，用自定义参数文件覆盖
`topics.cmd_vel` 即可。

默认启用的离散动作服务：

| 槽位 | Service | 手柄组合 | 键盘 |
|------|---------|----------|------|
| `action1` | `/joy/action1` | `LB + A` | `t` |
| `action2` | `/joy/action2` | `RB + A` | `g` |
| `action3` | `/joy/action3` | `Back + Start` | `x` |
| `action4` | `/joy/action4` | 未绑定 | `q` |

## 使用前权限设置

遥控包只直接访问手柄输入设备：

- 手柄：`/dev/input/js*`，由 `joy_node` 读取。
- 键盘：由 `remote_control_node` 从前台终端 stdin 读取，不再读取 `/dev/input/event*`，也不需要 `input` 组权限。

先检查设备是否存在、当前用户是否有权限：

```bash
ls -l /dev/input/js*
groups
```

多数桌面登录会通过 udev/logind 给当前会话授予输入设备 ACL；SSH、Docker、
systemd、非桌面会话下通常没有这些权限。如果 `joy_node` 无法打开手柄设备，
把当前用户加入 `input` 组：

```bash
sudo usermod -aG input "$USER"
```

然后退出当前登录会话并重新登录。只想让当前 shell 临时生效时可运行：

```bash
newgrp input
```

验证：

```bash
id -nG
```

不要直接用 `sudo ros2 launch ...` 运行遥控节点。这样会切换 ROS 环境、日志目录
和用户会话权限，通常会制造更多问题。

节点不会独占全局键盘，因此不会影响其他终端或窗口；只有运行节点的前台终端
收到按键时，键盘遥控才会响应。

## 使用前环境设置

依赖：

- ROS2 Humble：`/opt/ros/humble/setup.bash`
- `colcon`
- `ros-humble-joy`：提供 `joy_node`
- `launch` / `launch_ros`：提供 ROS launch 启动能力

缺少 `joy_node` 时安装：

```bash
sudo apt install ros-humble-joy
```

从仓库根目录构建并 source overlay：

```bash
source /opt/ros/humble/setup.bash
./build.sh
source build/ros_ws/install/setup.bash
```

每个新终端都需要重新 source ROS2 和仓内 overlay。运行仿真端和遥控端的终端
需要处于同一个 ROS domain；如果设置过 `ROS_DOMAIN_ID`，两边必须一致。

受限环境中如果 ROS 无法写入默认日志目录，可以把日志目录放到 `/tmp`：

```bash
export ROS_HOME=/tmp/ausim_ros_home
export ROS_LOG_DIR=/tmp/ausim_ros_log
```

## 启动方式

默认推荐使用 Qt5 图形窗口，获得真实多键键盘状态、窗口焦点隔离和可视化状态面板：

```bash
source /opt/ros/humble/setup.bash
source build/ros_ws/install/setup.bash
ros2 launch remote_control remote_control_gui.launch.py
```

也可以从仓库根目录一键启动，默认就是 GUI 模式：

```bash
./run_remote_control.sh
```

默认 GUI 模式会尝试启动 `joy_node`；有手柄并产生 `/joy` 消息时优先手柄，没有
新鲜 `/joy` 消息时回退窗口键盘。禁用 `joy_node` 时使用：

```bash
./run_remote_control.sh --use-joy false
ros2 launch remote_control remote_control_gui.launch.py use_joy:=false
```

如果需要最低限度非 GUI 支持，使用 `--headless` 启动传统终端节点：

```bash
./run_remote_control.sh --headless
```

也可以手动运行非 GUI 节点。此模式只适合前台终端键盘输入，且终端输入没有真实按键释放事件：

```bash
source /opt/ros/humble/setup.bash
source build/ros_ws/install/setup.bash
ros2 run remote_control remote_control_node --ros-args --params-file third_party/ros_ws/src/remote_control/config/xbox_like.yaml
```

可选参数：

```bash
ros2 launch remote_control remote_control_gui.launch.py device_id:=0
ros2 launch remote_control remote_control_gui.launch.py config:=/absolute/path/to/xbox_like.yaml
ros2 launch remote_control remote_control_gui.launch.py use_joy:=false
```

GUI launch 行为：

1. 启动 `remote_control_gui_node` 可执行文件，ROS 节点名为 `remote_control_node`，参数文件默认来自已安装 overlay 中的 `config/xbox_like.yaml`。
2. launch 会额外传入 `gui.config_path`，GUI 的“应用并保存”会写回本次启动的 YAML。
3. `use_joy:=true` 时启动 `joy_node`，参数为 `device_id`、`deadzone=0.05`、`autorepeat_rate=20.0`。
4. `use_joy:=false` 时不启动 `joy_node`，GUI 只使用窗口键盘输入。
5. 有新鲜 `/joy` 消息时手柄优先；没有 `/joy` 输入时 GUI 回退窗口键盘。

非 GUI launch 仍保留给 `--headless` 和兼容场景：

```bash
ros2 launch remote_control remote_control.launch.py
```

## 默认键盘映射

键盘读取基于前台终端 stdin。`Ctrl+C` 仍由终端处理并正常退出节点；未选中该终端时，
其他窗口中的按键不会传给遥控节点。由于终端输入没有按键释放事件，运动键会按
`keyboard.key_timeout` 短暂锁存，长按时依赖终端自动重复刷新。

Qt5 图形窗口使用窗口焦点键盘事件，支持真实 key press/release 和多键同时按下；
窗口失焦时会立即清空键盘状态并发布零速度。顶部输入来源只显示当前生效来源
（手柄 / 键盘 / 无输入）；键盘状态不再单独占用一行。

GUI 还提供在线配置面板：

- 手柄输入状态：收到 `/joy` 后显示全部 `axes[]` 和 `buttons[]`；轴在左侧显示为“编号在上、竖直状态条居中、数值在下”，按钮在右侧显示为“编号在上、红/绿圆点居中”并自适应换行，不再显示“轴”“按钮”分区标题。
- `cmd_vel` 最大输出：在线调整 `linear.x/y/z` 和 `angular.z`，手柄与键盘速度同步更新；保存时写回 `scale.*` 与 `keyboard.scale.*`。
- 语言切换：点击 `English` / `中文` 即时切换界面提示文字，保存时写回 `gui.language`。
- 手柄到 action 映射：默认显示 `action1..action6` 6 个槽位，可编辑 service、按钮组合和键盘快捷键；“捕获”会读取当前按下的手柄按钮。

| 键 | 行为 |
|----|------|
| `w` / `s` | 前进 / 后退 |
| `a` / `d` | 左 / 右平移 |
| `r` / `f` | 上 / 下 |
| `j` / `l` | 偏航左 / 右 |
| `space` | 立即清零键盘运动状态 |
| `t` | 调用 `action1` |
| `g` | 调用 `action2` |
| `x` | 调用 `action3` |
| `q` | 调用 `action4` |

## 自定义配置

动作槽位在 `config/xbox_like.yaml` 的 `actions:` 块里配置。GUI 默认编辑
`action1..6` 这 6 个槽位：

```yaml
actions:
  action5:
    service: /joy/action5
    buttons: [3]
    keyboard: "c"
```

`service` 为空会禁用该槽位。禁用摇杆组合时不要写 `buttons: []`；ROS2 参数
系统对空数组类型不稳定，默认配置使用 `buttons: [-1]` 作为未绑定哨兵值。

## 常见问题

- `Package 'remote_control' not found`：当前终端没有 `source build/ros_ws/install/setup.bash`，或还没有运行 `./build.sh`。
- `joy_node` 找不到或启动失败：确认已安装 `ros-humble-joy`，并检查 `/dev/input/js*` 是否存在。
- 键盘不可用：确认用前台终端运行 `remote_control_node`；如果通过 `ros2 launch` 启动且 stdin 不是 TTY，键盘模式会自动禁用。
- 动作键有日志但 service not ready：仿真桥或接收端还没有启动对应 `/joy/actionN` service。
- 手柄能输入但仿真不动：确认 `topics.cmd_vel` 是否发布到仿真桥订阅的 topic；仓内默认遥控入口通常是 `/joy/cmd_vel`。
