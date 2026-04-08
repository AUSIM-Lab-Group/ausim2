# -------------------------------------------------------------
# Library: aima::sim::module::publisher
# -------------------------------------------------------------

aimrte_library(
  DEPEND
  aima::sim::data   
  aima::sim::converts
  aima::sim::common::db
  aima::sim::module::mujoco
  aima::sim::manager::aimrt
  aima::sim::manager::publisher
  IMPORT
  grid_map_msgs
  grid_map_ros
  grid_map_pcl
  pcl_conversions
  tf2_msgs)

