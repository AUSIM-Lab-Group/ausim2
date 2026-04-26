# `xbox_like.yaml` 配置解析

## 概览

`xbox_like.yaml` 用于驱动 `remote_control_node`。它会将摇杆和键盘输入映射为：

- 发布到 `/joy/cmd_vel` 的连续运动控制输出
- 触发到 `/joy/action1..6` 的离散动作命令

节点本身并不理解机器人特有的行为。它只负责将遥控输入转换为标准运动指令，以及一组可配置的固定动作服务调用。

## 字段逐项说明

### `topics`

- `topics.joy`
  - `sensor_msgs/msg/Joy` 的输入话题
  - 节点订阅该话题以读取摇杆轴和按钮信息

- `topics.cmd_vel`
  - `geometry_msgs/msg/Twist` 的输出话题
  - 节点会以固定频率向该话题发布运动指令

### 核心运动参数

- `publish_rate_hz`
  - 用于发布 `cmd_vel` 的定时器频率
  - 必须为正数
  - 数值越高，输出延迟越低，但发布频率也越高

- `joy_timeout`
  - 上一次摇杆消息在被视为过期前允许的最大时间间隔
  - 如果超过该超时时间，节点会在可用时切换到键盘后备控制；否则发布零速度

- `deadzone`
  - 轴输入死区阈值
  - 如果 `abs(axis_value) < deadzone`，该轴输入会被视为 0
  - 用于抑制摇杆回中附近的小幅噪声

- `require_enable_button`
  - 当为 `true` 时，只有在启用按钮当前被按下的情况下，才会生成摇杆运动指令

- `enable_button`
  - 用作摇杆运动使能门控的按钮索引
  - 仅在 `require_enable_button` 为 `true` 时生效

### 轴映射

- `axes.linear.x`
  - 用于前后运动的摇杆轴索引

- `axes.linear.y`
  - 用于左右平移的摇杆轴索引

- `axes.linear.z`
  - 用于上下运动的摇杆轴索引

- `axes.angular.yaw`
  - 用于偏航角速度的摇杆轴索引

这些值都是原始的 `Joy.axes[]` 索引，因此具体取值取决于控制器布局以及 `joy_node` 的输出顺序。

### 运动缩放

- `scale.linear.x`
  - 应用于线速度 x 的轴缩放系数

- `scale.linear.y`
  - 应用于线速度 y 的轴缩放系数

- `scale.linear.z`
  - 应用于线速度 z 的轴缩放系数

- `scale.angular.yaw`
  - 应用于偏航的轴缩放系数

节点计算方式如下：

- `command.linear.x = deadzoned_axis(axes.linear.x) * scale.linear.x`
- `command.linear.y = deadzoned_axis(axes.linear.y) * scale.linear.y`
- `command.linear.z = deadzoned_axis(axes.linear.z) * scale.linear.z`
- `command.angular.z = deadzoned_axis(axes.angular.yaw) * scale.angular.yaw`

修改符号会反转方向。修改绝对值大小会改变输出灵敏度。

Qt5 GUI 的“cmd_vel 最大输出”会同步写回 `scale.*` 与 `keyboard.scale.*`。
如果原缩放系数为负，GUI 保存时会保留方向符号，只更新绝对值。

### `gui`

- `gui.language`
  - GUI 界面提示语言
  - 支持 `zh` 和 `en`
  - GUI 中点击 `English` / `中文` 可即时切换，并通过“应用并保存”写回配置
- `gui.window`
  - GUI 默认窗口宽高，由“应用并保存”写回当前窗口尺寸
- `gui.sections`
  - `cmd_vel_mapping_expanded` 和 `action_mapping_expanded` 控制两个映射区启动时是否展开
  - GUI 中折叠/展开映射区后点击“应用并保存”会写回当前状态

### `actions`

`actions` 块定义了离散动作槽位。默认 GUI 编辑 `action1..6` 这 6 个槽位；每个槽位都可以由摇杆按钮、键盘按键或二者共同触发。

对于每个 `actionN`：

- `service`
  - 目标 `std_srvs/srv/Trigger` 服务名
  - 如果为空，该动作槽位会被完全忽略

- `buttons`
  - 触发该动作的摇杆按钮组合
  - 列出的所有按钮必须同时按下
  - `[-1]` 被用作“未绑定摇杆按钮”的哨兵值
  - 代码在加载时会移除 `-1`，从而使该动作没有摇杆组合绑定

- `keyboard`
  - 用于触发同一动作的单个键盘按键
  - 空字符串表示没有键盘绑定

当前默认配置：

- `action1`：服务 `/joy/action1`，按钮 `[4, 0]`，键盘 `t`
- `action2`：服务 `/joy/action2`，按钮 `[5, 0]`，键盘 `g`
- `action3`：服务 `/joy/action3`，按钮 `[6, 7]`，键盘 `x`
- `action4`：服务 `/joy/action4`，无摇杆组合，键盘 `q`
- `action5..6`：禁用

### 离散命令安全参数

- `command_cooldown`
  - 任意两次离散动作触发之间的最小时间间隔
  - 防止组合键或按键重复按下时过快连续触发

- `motion_suppress_after_command`
  - 离散动作触发后抑制运动输出的持续时间
  - 节点会立即发布一次零 `cmd_vel`，并在这段时间内持续输出零运动
  - 这样可以减少连续运动与服务触发行为之间的重叠

### `keyboard`

- `keyboard.enabled`
  - 启用或禁用键盘后备控制
  - 键盘控制仅在 stdin 是 TTY 时可用

- `keyboard.key_timeout`
  - 单次按下运动键后，该键保持激活状态的持续时间
  - 终端中的运动键并不是通过持续按住状态来维持，而是在每次按键后锁存一个短暂时间窗口

- `keyboard.scale.linear.x`
  - 键盘控制下固定的线速度 x 幅值

- `keyboard.scale.linear.y`
  - 键盘控制下固定的线速度 y 幅值

- `keyboard.scale.linear.z`
  - 键盘控制下固定的线速度 z 幅值

- `keyboard.scale.angular.yaw`
  - 键盘控制下固定的偏航幅值

键盘运动按键如下：

- `w` / `s`：前进 / 后退
- `a` / `d`：左移 / 右移
- `r` / `f`：上升 / 下降
- `j` / `l`：向左偏航 / 向右偏航
- `space`：立即清除当前键盘运动状态

## 运行时行为总结

节点运行在固定频率的发布循环中。

1. 处理待执行的键盘动作事件。
2. 使用 `joy_timeout` 判断摇杆输入是否仍然新鲜。
3. 如果运动抑制处于激活状态，则发布零速度。
4. 否则：
   - 如果摇杆输入仍然有效，则根据摇杆轴构造一个 `Twist`。
   - 否则如果键盘可用，则根据键盘状态构造一个 `Twist`。
   - 否则发布零速度。
5. 离散动作则通过 `Trigger` 服务独立触发。

这意味着该配置文件同时组合了三部分内容：

- 话题连接配置
- 连续运动映射
- 离散动作映射

## 当前配置启用了什么

当前文件提供了：

- 基于摇杆的 4 轴连续控制
- 当摇杆消息过期时可选的键盘后备控制
- 4 个已启用的离散动作服务槽位
- 2 个保留未启用槽位，用于后续扩展
- GUI 默认中文界面，可切换并保存为英文
- 离散命令的速率限制
- 离散动作后的临时运动抑制

## 哪些内容可以安全修改

以下字段属于常规可定制项：

- `topics` 下的话题名
- `axes` 下的控制器轴索引
- `scale` 下的方向和灵敏度
- 通过 `require_enable_button` 和 `enable_button` 控制安全门行为
- `actions.*.service` 下的服务端点
- `actions.*.buttons` 下的摇杆按钮组合
- `actions.*.keyboard` 下的键盘快捷键
- `gui.language` 下的 GUI 语言
- 时序参数：
  - `publish_rate_hz`
  - `joy_timeout`
  - `command_cooldown`
  - `motion_suppress_after_command`
  - `keyboard.key_timeout`
- `keyboard.scale` 下的键盘输出幅值

典型使用场景：

- 适配另一种手柄布局
- 通过将缩放系数设为负值来反转摇杆方向
- 启用更多动作槽位
- 让键盘后备模式比摇杆模式更慢或更快
- 在运动前要求按住安全使能按钮

## 约束与注意事项

- `publish_rate_hz` 必须大于 `0`
- `joy_timeout` 必须为非负数
- `keyboard.key_timeout` 必须大于 `0`
- 无效轴索引不会报错；它们只会产生 `0.0`
- 空的 `service` 会完全禁用该动作槽位
- 仓库中刻意避免使用 `buttons: []`，因为 ROS 参数系统对空数组的类型处理可能比较别扭；因此使用 `[-1]` 代替
- 键盘后备控制要求前台 TTY；在非交互式启动环境中会自动禁用键盘模式

## 可优化点

### 1. 让离散动作真正动态化

当前实现中，C++ 代码将 `action1..8` 写死。

可改进方向：

- 支持从参数中加载任意长度的动作列表，而不是固定编号槽位

收益：

- 更易于扩展
- 更少模板化代码
- 降低配置结构与代码实现之间的耦合

### 2. 改进死区处理

当前死区逻辑是硬阈值裁剪：

- 低于阈值 -> `0`
- 高于阈值 -> 保持原始值不变

可改进方向：

- 先应用死区，再将剩余范围重新缩放回 `[0, 1]`

收益：

- 在摇杆中心附近提供更平滑的低速控制
- 减少死区边界处的突变感

### 3. 增加配置校验

当前节点只校验了：

- `publish_rate_hz` 为正
- `joy_timeout` 非负

可改进方向：

- 在收到第一条 `Joy` 消息时，根据实际摇杆维度校验轴索引
- 对可疑的缩放系数大小给出警告
- 当键盘绑定与保留运动键冲突时给出警告

收益：

- 更容易调试
- 减少静默配置错误

### 4. 更丰富的动作触发语义

当前动作触发仅支持：

- 所有按钮同时按住的摇杆组合
- 单个键盘按键触发

可改进方向：

- 支持边沿触发和电平触发模式
- 支持长按
- 为每个动作单独设置冷却时间
- 可选要求动作触发时也必须按下使能按钮

收益：

- 遥控交互设计更灵活
- 减少下游逻辑承载过多行为语义的压力

### 5. 进一步明确 YAML 内联注释

当前配置已经具备可读性，但有两点值得在注释中进一步强调：

- `[-1]` 是“未绑定摇杆按钮”的哨兵值
- `action4` 当前默认仅支持键盘触发

收益：

- 编辑时更不容易困惑
- 降低误配置概率

### 6. 重新审视超时参数调优

`joy_timeout: 0.5` 只有在上游摇杆发布稳定时才算合理。

可改进方向：

- 结合 `joy_node autorepeat_rate` 一起调优 `joy_timeout`

收益：

- 避免在摇杆与键盘模式之间出现意外抖动切换

## 推荐的实用调优建议

对于日常使用，最有价值的配置层改进包括：

1. 通过实际 `ros2 topic echo /joy` 输出核对所有轴索引
2. 在 `actions` 块旁边记录真实控制器的按钮布局
3. 如果担心误碰摇杆，启用 `require_enable_button`
4. 根据实际 `joy_node` 重复发布行为调整 `joy_timeout`
5. 只有在确实存在下游服务要绑定时，才定义 `action5..8`

## 源码参考

- `third_party/remote_control/config/xbox_like.yaml`
- `third_party/remote_control/src/remote_control_node.cpp`
- `third_party/remote_control/src/remote_control_node.hpp`
- `third_party/remote_control/README.md`
