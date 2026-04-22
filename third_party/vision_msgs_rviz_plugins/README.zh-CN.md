# vision_msgs_rviz_plugins

该包包含一个用于 ROS 2 的 RVIZ2 插件，用于显示 vision_msgs 消息。

- [x] Detection3DArray
  - [x] 显示 ObjectHypothesisWithPose/score
  - [x] 根据 ObjectHypothesisWithPose/id 切换颜色 [car: orange, person: blue, cyclist: yellow, motorcycle: purple, other: grey]
  - [x] 可视化属性
    - [x] 透明度（Alpha）
    - [x] 线框或盒体（Line or Box）
    - [x] 线宽（Linewidth）
    - [x] 基于提供的 yaml 文件切换颜色映射
- [x] Detection3D
  - [x] 显示 ObjectHypothesisWithPose/score
  - [x] 根据 ObjectHypothesisWithPose/id 切换颜色 [car: orange, person: blue, cyclist: yellow, motorcycle: purple, other: grey]
  - [x] 可视化属性
    - [x] 透明度（Alpha）
    - [x] 线框或盒体（Line or Box）
    - [x] 线宽（Linewidth）
    - [x] 基于提供的 yaml 文件切换颜色映射
- [x] BoundingBox3D
    - [x] 透明度（Alpha）
    - [x] 线框或盒体（Line or Box）    
        <span style="color:red">**由于 vision_msgs/BonudingBox 可视化中没有 header，TF 变换将使用 rviz 的固定坐标系（fixed frame）**</span>
    - [x] 线宽（Linewidth）
- [x] BoundingBox3DArray
    - [x] 透明度（Alpha）
    - [x] 线框或盒体（Line or Box）
    - [x] 线宽（Linewidth）

![Bounding Box Array](assets/BBoxArray.gif)
