# `xbox_like.yaml` GUI 使用说明

这份配置用于 `remote_control` 的默认手柄/键盘遥控界面。日常使用优先打开 Qt5 GUI，
通过窗口查看手柄输入、调整 `cmd_vel` 映射、绑定 action，并把常用设置保存回
`xbox_like.yaml`。

## 启动 GUI

从仓库根目录启动，默认就是 GUI 模式：

```bash
./run_remote_control.sh
```

也可以手动启动 ROS2 launch：

```bash
source /opt/ros/humble/setup.bash
source build/ros_ws/install/setup.bash
ros2 launch remote_control remote_control_gui.launch.py
```

默认会同时启动 `joy_node` 读取手柄。只用 GUI 键盘控制，或指定设备/配置时：

```bash
./run_remote_control.sh --use-joy false
ros2 launch remote_control remote_control_gui.launch.py use_joy:=false
ros2 launch remote_control remote_control_gui.launch.py device_id:=0
ros2 launch remote_control remote_control_gui.launch.py config:=/absolute/path/to/xbox_like.yaml
```

## GUI 基本行为

- 窗口有焦点时，键盘控制才生效；窗口失焦会清空键盘状态并输出零速度。
- 有新鲜 `/joy` 手柄输入时优先使用手柄；没有手柄输入时回退到窗口键盘。
- 顶部会显示当前输入来源：手柄、键盘或无输入。
- 点击 `English` / `中文` 可即时切换语言。
- 点击“应用并保存”会把 GUI 中的可编辑设置写回当前 YAML。

## 键盘控制

| 键 | 行为 |
|----|------|
| `w` / `s` | 前进 / 后退 |
| `a` / `d` | 左 / 右平移 |
| `r` / `f` | 上 / 下 |
| `j` / `l` | 向左 / 向右偏航 |
| `space` | 立即清零键盘运动状态 |

| 键 | 行为 |
|----|------|
| `t` | 触发 `action1` |
| `g` | 触发 `action2` |
| `x` | 触发 `action3` |
| `q` | 触发 `action4` |
| `b` | 触发 `action5` |

## 手柄状态显示

“手柄状态显示”区域会在收到 `/joy` 后显示所有轴和按钮。轴显示编号、竖向状态条
和值；按钮显示编号和彩色圆点，便于确认手柄布局。该区域可折叠，保存后下次启动
会恢复当前折叠状态。

## `cmd_vel` 映射

“手柄到 cmd_vel 映射”区域用于调整连续运动控制：

- `linear.x`：前进 / 后退
- `linear.y`：左 / 右平移
- `linear.z`：上 / 下
- `angular.z`：偏航

每一项都可以选择手柄轴、反转方向，并设置最大输出。GUI 保存时会同步更新手柄
和键盘的最大输出值；如果原配置方向为负，保存后仍会保留反向。

## Action 映射

“手柄到 action 映射”区域默认编辑 `action1..action6` 6 个槽位。每个槽位包括：

- `service`：触发后调用的服务名，例如 `/joy/action1`
- 手柄按钮组合：可手动填写，也可点击“捕获”读取当前按下的按钮
- 键盘快捷键：窗口有焦点时可触发同一个 action

不需要的槽位可以把 `service` 留空。没有手柄按钮绑定时，保持默认的 `-1` 占位即可。

## Action 历史

“Action 历史”区域显示已触发 action 的记录和接收状态。展开时可以查看完整历史；
折叠时标题栏会显示最后一次 action 的简要结果。该区域的展开/折叠状态也会被保存。

## 保存内容

点击“应用并保存”会写回：

- GUI 语言、窗口尺寸和各区域折叠状态
- 手柄轴到 `cmd_vel` 的映射、方向和最大输出
- `action1..action6` 的服务名、手柄按钮组合和键盘快捷键

如果只想临时试用，不要点击“应用并保存”，关闭窗口即可丢弃 GUI 中未保存的修改。

## 常见问题

- 找不到 `remote_control` 包：确认已运行 `./build.sh` 并 source 了 ROS overlay。
- GUI 没有手柄输入：检查手柄设备是否存在，并尝试修改 `device_id`。
- 键盘控制无效：确认 GUI 窗口处于前台焦点。
- action 有触发记录但没有响应：确认仿真或接收端已经提供对应 `/joy/actionN` 服务。
- 仿真不动：确认接收端订阅的是配置中的 `/joy/cmd_vel`。
