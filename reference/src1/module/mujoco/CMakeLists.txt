# -------------------------------------------------------------
# Library: aima::sim::module::mujoco
# -------------------------------------------------------------

aimrte_library(
  GEN_TOP_HEADER
  # -------------------------------------------------------------
  DEPEND
  aima::sim::data
  aima::sim::data
  aima::sim::common::db
  aima::sim::common::parameter
  aima::sim::manager::aimrt
  mujoco::mujoco
  mujoco
  mujoco::libmujoco_simulate
  glfw
  IMPORT
  rclcpp
  sensor_msgs
)
