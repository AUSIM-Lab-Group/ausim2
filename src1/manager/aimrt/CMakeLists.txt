# -------------------------------------------------------------
# Library: aima::sim::manager::aimrt
# -------------------------------------------------------------

aimrte_library(
  GEN_TOP_HEADER
  # -------------------------------------------------------------
  DEPEND
  aima::sim::common::db # db
  aimrt::interface::aimrt_module_cpp_interface # aimrt
  aimrt::interface::aimrt_module_ros2_interface)
