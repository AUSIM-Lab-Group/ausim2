# Remote Control

仓内 ROS2 RC 适配节点：把手柄 / 键盘输入转换成标准化的 `/joy/cmd_vel` 和一组 `/joy/actionN` service 调用。节点本身不关心机器人语义，只负责把 RC 输入映射到可配置接口。

- 订阅：`/joy`
- 发布：`/joy/cmd_vel`（`geometry_msgs/Twist`）
- 调用：`/joy/actionN`（`std_srvs/Trigger`）
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
| `t` | 调用 `action1`（默认 `/joy/action1`） |
| `g` | 调用 `action2`（默认 `/joy/action2`） |
| `x` | 调用 `action3`（默认 `/joy/action3`） |
| `q` | 调用 `action4`（默认 `/joy/action4`） |

## 默认手柄组合键

| 组合 | 动作槽位 |
|------|------|
| `LB + A`      | `action1` |
| `LB + B`      | `action2` |
| `Back + Start`| `action3` |
| `action4` | 默认未绑定按钮（仅键盘），需要在 YAML 里显式配置 |

注意：如果想禁用某个摇杆组合，不要写 `buttons: []`。ROS2 参数文件里空数组常会丢失类型，默认配置使用 `buttons: [-1]` 作为“未绑定”哨兵值。

默认配置里，`action1..4` 分别指向 `/joy/action1..4`。接收端通常再把它们映射成 `takeoff` / `land` / `reset` / `estop`。

## 自定义动作 / 按键

在 `config/xbox_like.yaml` 的 `actions:` 块中配置槽位即可：

```yaml
actions:
  action5:
    service: /joy/action5
    buttons: [3]
    keyboard: "c"
```

节点不认识动作语义——只知道调用哪个 service。接收端配置决定 `actionN` 对应的状态机事件或其他离散命令。

## 说明

- 键盘 fallback 依赖前台终端 TTY；无 TTY 场景（nohup / systemd / Docker without `-t`）下键盘自动禁用，只剩手柄
- 离散动作触发时会先发布一次零速 `cmd_vel`，并按 `motion_suppress_after_command` 暂停运动输出，再异步调用目标 service
