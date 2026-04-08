# -------------------------------------------------------------
# Library: aima::sim::converts
# -------------------------------------------------------------

# find_package(tf2_msgs REQUIRED)

aimrte_library(
  GEN_TOP_HEADER
  # -------------------------------------------------------------
  DEPEND
  aima::sim::data
  aimdk::protocol::ROS
  IMPORT
  std_msgs
  sensor_msgs
  geometry_msgs
  # grid_map 相关依赖
  grid_map_core      # 核心库（最基础）
  grid_map_msgs      # 消息包（依赖 grid_map_core）
  grid_map_ros       # ROS 接口（依赖 grid_map_msgs 和 grid_map_core）
  grid_map_pcl       # PCL 转换（依赖 grid_map_ros 和 grid_map_msgs）
  pcl_conversions    # PCL 与 ROS 消息转换
  PCL
  tf2_msgs)
