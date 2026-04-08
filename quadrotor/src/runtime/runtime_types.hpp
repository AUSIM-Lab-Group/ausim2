#pragma once

#include <chrono>
#include <string>

#include <Eigen/Core>

#include "controller/se3_controller.hpp"
#include "controller/state.hpp"

namespace quadrotor {

struct ImuData {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
  Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d linear_acceleration = Eigen::Vector3d::Zero();
  bool has_linear_acceleration = false;
};

struct RuntimeInput {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  double sim_time = 0.0;
  double gravity_magnitude = 9.81;
  State current_state;
  ImuData imu;
};

struct VelocityCommand {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d linear = Eigen::Vector3d::Zero();
  Eigen::Vector3d angular = Eigen::Vector3d::Zero();
  std::chrono::steady_clock::time_point received_time = std::chrono::steady_clock::now();
};

struct GoalReference {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  State state;
  Eigen::Vector3d forward = Eigen::Vector3d(1.0, 0.0, 0.0);
  SE3Controller::ControlMode control_mode = SE3Controller::ControlMode::kPosition;
  std::string source = "demo";
};

struct RuntimeOutput {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  GoalReference goal;
  ControlCommand command;
  Eigen::Vector4d motor_speed_krpm = Eigen::Vector4d::Zero();
};

struct TelemetrySnapshot {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  double sim_time = 0.0;
  State state;
  ImuData imu;
  State goal_state;
  Eigen::Vector3d forward = Eigen::Vector3d(1.0, 0.0, 0.0);
  Eigen::Vector4d motor_speed_krpm = Eigen::Vector4d::Zero();
  std::string goal_source;
  bool has_goal = false;
};

}  // namespace quadrotor
