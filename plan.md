# dynamic_obs_generator 两项改进 — 子 agent prompt

本文件包含两份完整、可独立投递的子 agent prompt,分别对应 `third_party/dynamic_obs_generator/` 的两项改进:

1. **Task 1** — 修复动态障碍物与真实仿真设备的碰撞处理
2. **Task 2** — 自动发布 `/dyn_obstacle`(`ausim_msg/BoundingBox3DArray`)话题,发布频率在 YAML 中可配

两份 prompt 的"约束"章节已严格划分字段边界(`collision_enabled` vs `publish.*`),避免并行执行时互相覆盖。

---

## 共同背景(两个 agent 都需要知道)

- `DynamicObstacleManager` 每步直接写 `model->geom_pos[geom_id*3 + {0,1,2}]` 做轨迹播放(`third_party/dynamic_obs_generator/dynamic_obstacle_manager.cpp:436-438`),不走 MuJoCo 物理积分
- `obstacle_config.cpp` 目前 **不解析** `collision_enabled` 字段;C++ 侧把每个障碍物是否 collidable 通过 `model_->geom_contype / geom_conaffinity` 推断(`dynamic_obstacle_manager.cpp:172-173`)
- Python 生成脚本 `generate_scene_obstacles.py:405-418` 会根据 YAML 的 `collision_enabled` 给 geom 写 `contype/conaffinity`
- 障碍物当前以裸 `<geom>` 形式注入 `<worldbody>`(`generate_scene_obstacles.py:405-418`),没有用 mocap body 包裹
- C++ 外层封装 `ausim::DynamicObstacleRuntime`(`ausim_common/src/sim/dynamic_obstacle_runtime.{hpp,cpp}`)由 `quadrotor_sim.cpp:1368-1379` 和 `scout_sim.cpp:681-689` 持有并每步调用 `PrepareForStep`
- `DynamicObstacleConfig`(enabled / config_path)解析在 `ausim_common/src/config/quadrotor_config.cpp:307-311`
- Telemetry / IPC 路径: sim 端写 `runtime.telemetry_snapshot` → bridge 端 `TelemetryLoop` 读出并推给 publisher vector(`ausim_common/src/manager/ros_bridge_process_manager.cpp:233-236`)
- `TelemetryPacket` 是 `std::is_trivially_copyable` POD(`ausim_common/src/converts/ipc/bridge_packets.hpp:55`),**不能**直接塞变长障碍物数组
- ROS publisher 接口 `ITelemetryPublisher::Publish(const ipc::TelemetryPacket&)`(`ausim_common/src/ros/publisher/i_telemetry_publisher.hpp:7-11`)
- 全局 publisher(clock / robot_mode)在 `ausim_common/src/ros/ros2_bridge.cpp` 的 `BuildPublishers()` 顶层创建,并非 `sensors[]` 循环里
- ROS bridge 统一由 `ros2_bridge.cpp` 里一个 `wall_timer` 按 `ros2.publish_rate_hz` 驱动(`ros2_bridge.cpp:353`)
- `third_party/ausim_msg/msg/BoundingBox3DArray.msg` 已定义且已加入 `CMakeLists.txt` 的 `rosidl_generate_interfaces`:

  ```
  std_msgs/Header header
  ausim_msg/BoundingBox3D[] boxes      # each: geometry_msgs/Pose center + geometry_msgs/Vector3 size
  ```

---

## Task 1 — Collision handling for real devices

**问题**:用户报告"障碍物和真实仿真设备的碰撞处理没有处理好"。根因是当 `collision_enabled: true` 时,Python 给 geom 写 `contype=1 conaffinity=1`,但 C++ 每步仍然直接 `model->geom_pos[...] = ...` 做 teleport。在 MuJoCo 中,对一个 collidable geom 在物理步外 teleport 会违反接触求解的连续性假设——机体可能穿透、接触力爆炸、solver 不稳定。MuJoCo 的正统做法是用 `mocap` body 包裹 geom,以 `data->mocap_pos / mocap_quat` 做运动学驱动;mocap body 不响应外力但正常参与接触求解。

### 子 agent 1 prompt(完整、可直接投递)

```
你要改进 ausim2 仓库 `third_party/dynamic_obs_generator/` 的动态障碍物与真实仿真设备的碰撞处理。当前实现会让障碍物在启用碰撞时 teleport 穿透机体,导致接触力爆炸与 solver 不稳定。

## 背景(已调研,可直接采信)

1. Python 生成脚本 `third_party/dynamic_obs_generator/generate_scene_obstacles.py:405-418` 会读 YAML 的 `collision_enabled`,给每个障碍物写入 `<geom ... contype="1" conaffinity="1">`,直接挂在 `<worldbody>` 下
2. C++ 侧 `third_party/dynamic_obs_generator/dynamic_obstacle_manager.cpp:121-162` 扫描所有 `dynamic_obs_*` 前缀的 geom;`RuntimeObstacle::collidable` 由 `model_->geom_contype/geom_conaffinity` 推断(行 172-173)
3. 每步更新位置的路径是 `ApplyTrajectory` → `ApplyTrajectoryRange` → `ApplySingleObstacleTrajectory`(行 374-439),最终是**直接写** `model_->geom_pos[addr+0/1/2] = new_x/y/z`。对 collidable geom 做这种 teleport 就是 bug 的根本
4. `ObstacleConfig`(`third_party/dynamic_obs_generator/obstacle_config.{hpp,cpp}`)目前完全不解析 `collision_enabled` 字段,C++ 一侧对用户意图是瞎的,只能靠 XML 属性反推
5. C++ 外部封装类 `ausim::DynamicObstacleRuntime`(`ausim_common/src/sim/dynamic_obstacle_runtime.cpp`)已经把 `PrepareForStep` 接入两种机体(quadrotor/scout)的 sim 循环
6. 机体 MJCF 样例:`assets/crazyfile/*.xml`、`assets/scout_v2/*.xml`。真实机体的 collidable body 会参与接触

## 要求

1. **实现 mocap body 方案**:修改 `generate_scene_obstacles.py` 的 `inject_obstacles`,当 `collision_enabled=True` 时,不再把 geom 直接挂在 `<worldbody>` 下,而是生成:

   ```xml
   <body name="dynamic_obs_<i>" mocap="true" pos="<x y z>">
     <geom name="dynamic_obs_<i>_geom" type="..." size="..." contype="1" conaffinity="1" rgba="..."/>
   </body>
   ```

   即 body 带 `mocap="true"`,geom 为它的子元素并继承 `dynamic_obs_` 命名约定。`collision_enabled=False` 时保持现在裸 geom 的老行为(纯视觉,无 mocap 开销)。

2. **更新 `ObstacleConfig`**:在 `obstacle_config.hpp/cpp` 增加 `bool collision_enabled = false;` 字段,在 `LoadConfigFromYaml` 中 `AssignIfPresent(root, "collision_enabled", &config.collision_enabled);`。写入 `IsValid` / `GetDebugInfo`。**注意**:字段名严格为 `collision_enabled`,与 Python 端一致。

3. **扩展 `DynamicObstacleManager::ScanSceneObstacles`**:

   - 对每个候选 obstacle,先看是否存在同名(或 `<name>_geom` 子 geom 的父 body)`mocap=true` body。若存在,则走 "mocap path";否则走原有 "direct geom_pos path"
   - 在 `RuntimeObstacle` 上增加字段:`int mocap_body_id = -1;`、`int mocap_addr = -1;`(`model_->body_mocapid[body_id] * 3`)、`bool is_mocap = false;`
   - 要兼容 Python 端的命名约定:body 名是 `dynamic_obs_<i>`,其下的 geom 名是 `dynamic_obs_<i>_geom`。Manager 扫描时 **优先扫 body 级**;若找到 mocap body,则对应 geom 记为 is_mocap 且把位置写 `data_->mocap_pos`
   - 对 **is_mocap=true** 的障碍:`ApplySingleObstacleTrajectory` 写 `data->mocap_pos[mocap_addr + 0..2] = new_xyz`(不写 `model->geom_pos`)。MuJoCo 不需要重新 `mj_forward` 来应用 mocap 更新;但如果已有 geom_pos 的 baked 值,需要保持一致
   - 对 **is_mocap=false** 的障碍:保持现在的 `geom_pos` 路径不变
   - `requires_physics_rate_updates_` 逻辑改为:`collidable && !is_mocap` 才需要 physics 节拍;mocap 路径可以保持 sensor 节拍,因为 mocap 写入不依赖物理步

4. **safety check**:在 `Initialize` 里,若发现任何障碍物既 collidable(contype/conaffinity 非 0)又 **不是** mocap(即裸 geom),打 clear 的 `std::cerr` warning 指出 "collidable dynamic_obs 必须用 mocap body 包裹,否则 teleport 会造成接触爆炸",并仍保留原 geom_pos 行为(不要 crash,因为可能是用户手写的 XML 且接受风险)

5. **保持向后兼容**:`collision_enabled` 缺省值 `false`,既有 YAML 不需改。`obstacle.yaml`(quadrotor 默认)保留 `collision_enabled: false`;`obstacle_scout.yaml` 当前写的是 `true`,对应生成的 XML 现在就会用 mocap body 路径

6. **文档**:同步更新 `third_party/dynamic_obs_generator/README.md` 的 "工作原理"/"限制" 章节,说明 mocap 路径与 direct path 的差异、什么时候走哪条

## 验证

- 构建:`./build.sh` 必须通过
- smoke:在 `obstacle_scout.yaml` 已有 `collision_enabled: true` 的情况下,先跑 `python3 third_party/dynamic_obs_generator/generate_scene_obstacles.py --sim-config ground_vehicle/cfg/sim_config.yaml --print-output-path`,检查生成的 `*.dynamic_obstacles.xml` 有 `<body mocap="true">` 包裹的 `dynamic_obs_*`
- 启动 `./em_run.sh -S`(选 `scout_v2`),障碍物运动时机体若与其碰撞,应看到正常的接触反作用而非 solver 爆炸 / 机体穿透飞出
- 反向回归:在 `obstacle.yaml` 保留 `collision_enabled: false` 的情况下启动 crazyfile,旧行为(纯视觉、geom_pos 路径)不变,障碍物继续移动,深度相机能看到

## 约束

- **不要动** ROS publisher/订阅/消息定义。ROS 发布部分由另一个 agent 负责,你的改动必须与它正交
- **不要** 新增 ObstacleConfig 里名为 `publish_*`、`publish`、`topic`、`frame_id`、`rate_hz` 的字段(那属于 ROS 任务的领地)。你的新字段只有 `collision_enabled`
- 保留 `DynamicObstacleManager::ApplyTrajectory` 的签名,不改动 `DynamicObstacleRuntime`(`ausim_common/src/sim/dynamic_obstacle_runtime.*`)的对外 API。只改内部实现
- 不引入新依赖
```

---

## Task 2 — 自动发布 `/dyn_obstacle` 话题

**问题**:用户希望动态障碍物生成器工作时,同时以 `ausim_msg/msg/BoundingBox3DArray` 类型发布 `/dyn_obstacle` 话题,发布频率在 YAML 里可配。

**设计关键**:

- 障碍物状态在 sim 进程(持有 MuJoCo model/data),ROS publish 在 bridge 进程,需要跨进程
- `TelemetryPacket` 是 POD,扩不了变长数组——需要新增一种 IPC packet
- 发布频率要 **独立** 于 `ros2.publish_rate_hz`(它是 telemetry 节拍),放 `obstacle.yaml` 里
- `/dyn_obstacle` 不隶属于某个传感器 → 属于 "全局 publisher",不应塞 `sensors[]` 循环;参考 `clock` publisher 的注册位置

### 子 agent 2 prompt(完整、可直接投递)

```
你要给 ausim2 仓库的动态障碍物生成器加一条 ROS publish 链路:当 `dynamic_obstacle.enabled=true` 时,自动以 `ausim_msg/msg/BoundingBox3DArray` 类型向 `/dyn_obstacle` 发布所有 `dynamic_obs_*` 的当前位姿与尺寸,发布频率由 `obstacle.yaml` 配置。

## 背景(已调研,可直接采信)

1. 消息定义已就绪:`third_party/ausim_msg/msg/BoundingBox3DArray.msg`:
   ```
   std_msgs/Header header
   ausim_msg/BoundingBox3D[] boxes   # 每个 BoundingBox3D: geometry_msgs/Pose center + geometry_msgs/Vector3 size
   ```
   该消息已加入 `third_party/ausim_msg/CMakeLists.txt` 的 `rosidl_generate_interfaces`。无需改 msg

2. C++ 配置入口 `DynamicObstacleConfig`(`ausim_common/src/config/quadrotor_config.cpp:307-311`)已解析 `dynamic_obstacle.enabled` / `config_path`,同时被 quadrotor 和 scout 共用

3. Sim 侧已持有 `ausim::DynamicObstacleRuntime`(`ausim_common/src/sim/dynamic_obstacle_runtime.cpp`),其内部的 `DynamicObstacleManager` 可以枚举每个 `RuntimeObstacle` 的位置与 half-size(`third_party/dynamic_obs_generator/dynamic_obstacle_manager.hpp:38-59`)

4. IPC 路径:sim → DataBoard(`runtime.telemetry_snapshot`)→ `RosBridgeProcessManager::TelemetryLoop`(`ausim_common/src/manager/ros_bridge_process_manager.cpp:233-236`)→ `TelemetryPacket` → bridge 进程 → `publishers_` 遍历。**`TelemetryPacket` 是 `std::is_trivially_copyable` POD(`ausim_common/src/converts/ipc/bridge_packets.hpp:55`),不能直接塞变长障碍物数组**

5. ROS publisher 接口:`ausim_common/src/ros/publisher/i_telemetry_publisher.hpp`。全局 publisher(非传感器)如 `ClockDataPublisher` 在 `ros2_bridge.cpp` 的 `BuildPublishers()` 顶层直接 push 入 `publishers_`,而不是在 `sensors[]` 循环里

6. 全局发布计时:`ros2_bridge.cpp:353` 的 `wall_timer(period_from publish_rate_hz)` 每 tick 扫 `publishers_`。这不适合 `/dyn_obstacle`——它需要自己独立的频率

## 要求

### (a) YAML schema 扩展

在 `third_party/dynamic_obs_generator/obstacle.yaml` 与 `obstacle_scout.yaml` 添加一个新子节(缺省禁用,保持向后兼容):

```yaml
publish:
  enabled: false
  topic: /dyn_obstacle
  frame_id: world
  rate_hz: 20.0
```

在 `obstacle_config.hpp/cpp` 的 `ObstacleConfig` 上增加:

```cpp
bool publish_enabled = false;
std::string publish_topic = "/dyn_obstacle";
std::string publish_frame_id = "world";
double publish_rate_hz = 20.0;
```

在 `LoadConfigFromYaml` 里解析 `publish:` 子节(参考现有 `range:` 子节的解析)。`IsValid` 要检查 `publish_rate_hz > 0`。**注意**:字段名定为 `publish_*`,以避开另一个 agent 负责的 `collision_enabled` 字段;两边不要重叠。

### (b) Sim 端:障碍快照

定义一个中立结构(放 `ausim_common/src/runtime/` 下新文件 `dynamic_obstacles_snapshot.hpp`):

```cpp
namespace ausim {
struct DynamicObstacleEntry {
  std::string name;
  double pos[3];
  double quat[4];    // w,x,y,z,留姿态但当前可固定为 identity
  double size[3];    // full-extent (不是 half-size)
};
struct DynamicObstaclesSnapshot {
  double sim_time = 0.0;
  std::vector<DynamicObstacleEntry> entries;
  std::string frame_id;
};
}
```

在 `DynamicObstacleRuntime` 加一个方法 `bool BuildSnapshot(DynamicObstaclesSnapshot& out) const;`,遍历 `manager_->moving_obstacles_`(你可能需要在 `DynamicObstacleManager` 上加一个访问器,或直接在 manager 上加 `FillSnapshot` 方法并转调)。Full-extent = half * 2。

在 `quadrotor/src/sim/quadrotor_sim.cpp` 和 `ground_vehicle/src/sim/scout_sim.cpp` 的主循环里,按 `publish_rate_hz` 的节拍(用 sim_time 做 rate-limit,参照现有 depth `compute_rate_hz` 的做法)调 `BuildSnapshot`,写入 DataBoard 新 slot `runtime.dynamic_obstacles_snapshot`。需要在 `ausim_common/src/runtime/data_board_interface.{hpp,cpp}` 加对应的 `Read/WriteDynamicObstaclesSnapshot`。

### (c) IPC packet

由于 TelemetryPacket 是 POD,**不要** 硬塞。在 `ausim_common/src/converts/ipc/bridge_packets.{hpp,cpp}` 新增一个 variable-length packet:

```cpp
struct DynObstaclePacketHeader {
  uint32_t magic;        // "DOBS"
  uint32_t entry_count;
  double   sim_time;
  char     frame_id[32];
};
struct DynObstaclePacketEntry {
  char    name[32];
  double  pos[3];
  double  quat[4];
  double  size[3];
};
```

packet 布局 = header + `entry_count` 个 entry 连续排布。提供:

- `ipc::ToDynObstaclePacket(const DynamicObstaclesSnapshot&, std::vector<uint8_t>& out)` 序列化
- `ipc::FromDynObstaclePacketBytes(const uint8_t* data, size_t len, DynamicObstaclesSnapshot& out)` 反序列化

在 `RosBridgeProcessManager`(`ausim_common/src/manager/ros_bridge_process_manager.cpp`)的 IPC 发送路径里扩展一条支路:检测 DataBoard 的 `dynamic_obstacles_snapshot` slot 有新快照时,把序列化字节通过 socketpair 发过去。消息帧前加 1-byte 类型 tag 以区分 TelemetryPacket(若当前协议没有 tag,则加一个;注意兼容性,需同时改 sim 端和 bridge 端的读写)。bridge 端 decode 出 `DynamicObstaclesSnapshot`,dispatch 给新 publisher。

### (d) 新 publisher

新建 `ausim_common/src/ros/publisher/data/dyn_obstacle_data_publisher.{hpp,cpp}`。参考 `odom_data_publisher.{hpp,cpp}` 的结构,但 **不要** 继承 `ITelemetryPublisher`(因为它接收的是 `TelemetryPacket`)。给它一个单独的接口 `IDynObstaclePublisher`(只有 `Publish(const DynamicObstaclesSnapshot&)` 一个方法)。

构造签名:`(rclcpp::Node* node, const std::string& topic, const std::string& default_frame_id)`。

在 `Publish` 里:把 snapshot 转成 `ausim_msg::msg::BoundingBox3DArray`,`header.stamp` = snapshot.sim_time 对应的 ROS 时间(参照 `ClockDataPublisher` 的时间换算),`header.frame_id` = snapshot.frame_id 或 default。每个 entry 转 `BoundingBox3D`:`center.position` = pos,`center.orientation` = quat,`size` = `geometry_msgs/Vector3(size[0], size[1], size[2])`。

### (e) 在 bridge 注册

在 `ausim_common/src/ros/ros2_bridge.cpp` 的 `BuildPublishers()` 里,**如果** `config.dynamic_obstacle.publish.enabled`(即把 YAML 里 obstacle.yaml 的 publish 子节透传到 bridge 的配置路径;需要在 `DynamicObstacleConfig` 上也加对应字段,或让 bridge 自己从 obstacle yaml 路径再 load 一次——推荐前者,统一由 sim_config 解析链注入),就创建一个 `dyn_obstacle_publisher_` 成员,**不放进** `publishers_` vector(因为它接的是不同的 snapshot,不是 TelemetryPacket)。

在 IPC 接收循环里,当解包到 DynObstaclePacket 时,调 `dyn_obstacle_publisher_->Publish(snapshot)`。频率由 sim 端节流,bridge 只管来一个发一个,保证不自己重复加限流。

### (f) CMakeLists

`ausim_common/` 下的 CMakeLists 里加新源文件。ROS bridge core 依赖 `ausim_msg` 的 C++ 生成目标(现已依赖,确认即可)。Sim core **不依赖** rclcpp / ausim_msg(保持 `*_sim_core` 无 ROS 依赖原则);所有消息转换只出现在 publisher 的 cpp 里。

## 验证

1. `./build.sh` 通过
2. `./em_run.sh --headless`(crazyfile 或 scout,先把对应 obstacle yaml 改 `publish.enabled: true`)
3. 另开 shell: `source /opt/ros/humble/setup.bash && source build/ros_ws/install/setup.bash`
4. `ros2 topic hz /dyn_obstacle` → 频率应约等于 yaml 里的 `publish.rate_hz`
5. `ros2 topic echo --once /dyn_obstacle` → `boxes[]` 数量 = 生成的 `dynamic_obs_*` 数量;`size` 字段 = 实际 geom full-extent;`center.position` 随时间变化(取两个 echo 应看到位置变化)
6. 把 yaml 改回 `publish.enabled: false` 启动 → 话题完全不存在(不是 0 Hz,是未注册)
7. `publish_rate_hz` 改成 5.0 再跑,`ros2 topic hz` 应跟随

## 约束

- **不要** 改 `ausim_msg/msg/*`
- **不要** 动障碍物的 MuJoCo 几何生成 / teleport / mocap 逻辑(那是另一个 agent 的领地)。`collision_enabled` 字段由对方负责,你不要在 YAML / ObstacleConfig 里重复添加
- sim 核心库(`quadrotor_sim_core` / `scout_sim_core`)必须 **继续不依赖** rclcpp 和 ausim_msg。所有 ROS 类型出现点只能在 `ausim_common_ros_bridge_core` 及其下游
- 不要把 `/dyn_obstacle` 插进 `sensors[]` 循环——它是全局 publisher
- 发布频率只看 `obstacle.yaml` 的 `publish.rate_hz`,不要受 `ros2.publish_rate_hz` 影响
```

---

## 协调注意事项(主 session 集成时)

- 两个 agent 都会修改 `third_party/dynamic_obs_generator/obstacle_config.{hpp,cpp}` 和两份 `obstacle*.yaml`——agent 1 只加 `collision_enabled`,agent 2 只加 `publish.*`,字段互不重叠,最后主 session 做 merge review
- 两个 agent 都会读 `DynamicObstacleManager` 内部数据:agent 1 **改** `ApplySingleObstacleTrajectory` 的 per-obstacle 写回路径,agent 2 **只新增** 只读访问器 / FillSnapshot。`RuntimeObstacle` 结构体字段也会被双方扩展(agent 1 加 `mocap_body_id/mocap_addr/is_mocap`,agent 2 基本不加)——不会直接冲突,但需要 review 合入
- 集成顺序建议:先跑 agent 1(mocap 包装 + `collision_enabled` 解析),通过后跑 agent 2(publish 链路);若并行,主 session 最终手动合并 `obstacle_config.{hpp,cpp}` 与两份 yaml
- 最终验收:**启用 `collision_enabled` + `publish.enabled`**,跑 scout 场景,应同时看到:机体与障碍物正常接触不爆炸 + `/dyn_obstacle` 以配置频率发布

## 关键文件清单

**Task 1 改动(主要)**:

- `third_party/dynamic_obs_generator/generate_scene_obstacles.py`(`inject_obstacles`)
- `third_party/dynamic_obs_generator/obstacle_config.{hpp,cpp}`(新增 `collision_enabled`)
- `third_party/dynamic_obs_generator/dynamic_obstacle_manager.{hpp,cpp}`(扫描 mocap body / 写回路径二选一)
- `third_party/dynamic_obs_generator/README.md`
- `third_party/dynamic_obs_generator/obstacle.yaml` / `obstacle_scout.yaml`(文档注释)

**Task 2 改动(主要)**:

- `third_party/dynamic_obs_generator/obstacle_config.{hpp,cpp}`(新增 `publish_*`)
- `third_party/dynamic_obs_generator/obstacle.yaml` / `obstacle_scout.yaml`(新增 `publish:` 子节)
- `ausim_common/src/runtime/dynamic_obstacles_snapshot.hpp`(新建)
- `ausim_common/src/runtime/data_board_interface.{hpp,cpp}`(新增 slot 读写)
- `ausim_common/src/sim/dynamic_obstacle_runtime.{hpp,cpp}`(`BuildSnapshot`)
- `ausim_common/src/converts/ipc/bridge_packets.{hpp,cpp}`(DynObstaclePacket)
- `ausim_common/src/manager/ros_bridge_process_manager.{hpp,cpp}`(IPC 分路)
- `ausim_common/src/ros/publisher/data/dyn_obstacle_data_publisher.{hpp,cpp}`(新建)
- `ausim_common/src/ros/ros2_bridge.{cpp,hpp}`(BuildPublishers 注册)
- `ausim_common/src/config/quadrotor_config.{hpp,cpp}`(透传 publish 配置)
- 相关 `CMakeLists.txt`
- `quadrotor/src/sim/quadrotor_sim.cpp` 与 `ground_vehicle/src/sim/scout_sim.cpp`(节流写 snapshot)
