# Remote Control

仓内 ROS2 teleop 节点：把手柄 / 键盘输入转换成 `/uav1/cmd_vel` + `/uav1/teleop/event`（字符串事件），由 sim 侧状态机解释。

- 订阅：`/joy`
- 发布：`/uav1/cmd_vel`（`geometry_msgs/Twist`）
- 发布：`/uav1/teleop/event`（`std_msgs/String`，任意事件名字符串）
- 不再有任何 service fallback。状态机事件（takeoff / land / reset / estop / 自定义）全部走 `/teleop/event` 话题。
- 没有新鲜手柄输入且 stdin 是 TTY 时，自动切到键盘控制。

## 启动（唯一入口）

```bash
./third_party/remote_control/run.sh
```

脚本做的事（所有参数仓内自持）：

1. source `/opt/ros/humble/setup.bash`
2. 后台启动 `ros2 run joy joy_node`，参数 `device_id=0` / `deadzone=0.05` / `autorepeat_rate=20.0` 全部 inline
3. 前台运行 `build/bin/remote_control_node`，params-file 指向仓内 `config/xbox_like.yaml`
4. 退出时清理 `joy_node` 子进程

可选环境变量：

- `REMOTE_CONTROL_CONFIG=/path/to/other.yaml`：覆盖默认配置文件路径

## 外部依赖

- ROS2 Humble
- `ros-humble-joy`（只需要里面的 `joy_node` 二进制；脚本不再读取其 share 目录里的 `joy-params.yaml`）

不依赖 `launch_ros` / `ament_index_python` / `python3-launch`。

## 默认键盘映射

| 键 | 行为 |
|----|------|
| `w` / `s` | 前进 / 后退 |
| `a` / `d` | 左 / 右平移 |
| `r` / `f` | 上 / 下 |
| `j` / `l` | 偏航左 / 右 |
| `space` | 立即清零 |
| `t` | 发事件 `takeoff` |
| `g` | 发事件 `land` |
| `x` | 发事件 `reset` |
| `m` | 发事件 `mode_next` |
| `q` | 发事件 `estop` |

## 默认手柄组合键

| 组合 | 事件 |
|------|------|
| `LB + A`      | `takeoff` |
| `LB + B`      | `land` |
| `Back + Start`| `reset` |
| `mode_next` / `estop` | 默认未绑定按钮（仅键盘），需要在 YAML 里显式配置 |

注意：如果想禁用某个摇杆组合，不要写 `buttons: []`。ROS2 参数文件里空数组常会丢失类型，默认配置使用 `buttons: [-1]` 作为“未绑定”哨兵值。

## 自定义事件 / 按键

在 `config/xbox_like.yaml` 的 `events:` 块中添加条目即可：

```yaml
events:
  my_custom_event:
    buttons: [3]
    keyboard: "c"
```

节点不认识事件白名单——事件名只是字符串，由 sim 侧状态机 YAML 决定语义。

## 说明

- 键盘 fallback 依赖前台终端 TTY；无 TTY 场景（nohup / systemd / Docker without `-t`）下键盘自动禁用，只剩手柄
- 未配置 `topics.teleop_event` 启动时会直接 throw；不再有"silently drop"路径
