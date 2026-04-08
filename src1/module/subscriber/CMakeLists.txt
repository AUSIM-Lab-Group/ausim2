# -------------------------------------------------------------
# Library: aima::sim::module::subscriber
# -------------------------------------------------------------

aimrte_library(
  DEPEND
  aima::sim::common::db # db
  aima::sim::manager::aimrt # aimrt
  aima::sim::converts # convert
  aima::sim::module::mujoco # mujoco
  aima::sim::manager::subscriber)
