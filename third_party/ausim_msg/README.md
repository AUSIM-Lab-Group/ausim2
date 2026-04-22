# ausim_msg

`ausim_msg` 是 ausim2 在 `third_party/` 中维护的独立 ROS 2 接口包。

首版接口内容参考并移植自 [reference/vision_msgs-humble/vision_msgs](/home/x/mujoco/ausim2/reference/vision_msgs-humble/vision_msgs)，用于证明：

- 自定义消息可以和主仓一起构建
- 构建后可以通过标准 ROS overlay `source` 载入
- 主仓内其他目标可以像依赖标准消息包一样依赖它

当前只提供消息接口和一个轻量 helper 头：

- `msg/*.msg`
- `include/ausim_msg/create_aabb.hpp`

不包含 RViz plugin，不接入现有仿真话题流。

典型使用方式：

```bash
source /opt/ros/humble/setup.bash
./build.sh
source build/ros_ws/install/setup.bash
ros2 interface show ausim_msg/msg/Detection2D
```
