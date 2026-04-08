#include "runtime/goal_provider.hpp"

#include <algorithm>
#include <cmath>

namespace quadrotor {
namespace {

enum class ExampleMode {
  kSimple = 1,
  kCircular = 2,
};

double QuaternionToYaw(const Eigen::Quaterniond& quaternion) {
  const double siny_cosp =
      2.0 * (quaternion.w() * quaternion.z() + quaternion.x() * quaternion.y());
  const double cosy_cosp =
      1.0 - 2.0 * (quaternion.y() * quaternion.y() + quaternion.z() * quaternion.z());
  return std::atan2(siny_cosp, cosy_cosp);
}

Eigen::Vector3d ForwardFromYaw(double yaw) {
  return Eigen::Vector3d(std::cos(yaw), std::sin(yaw), 0.0);
}

}  // namespace

DemoGoalProvider::DemoGoalProvider(const QuadrotorConfig& config) : config_(config) {}

GoalReference DemoGoalProvider::Evaluate(const GoalContext& context) {
  GoalReference goal;
  goal.state.quaternion = Eigen::Quaterniond::Identity();
  goal.forward = config_.hover_goal.heading;
  goal.control_mode =
      static_cast<SE3Controller::ControlMode>(config_.simulation.control_mode);
  goal.source = "demo";

  const bool velocity_mode = goal.control_mode == SE3Controller::ControlMode::kVelocity;
  const CircleTrajectoryConfig& circle = config_.circle_trajectory;
  if (velocity_mode && context.sim_time < circle.wait_time) {
    goal.state.position = config_.hover_goal.position;
    goal.state.velocity = Eigen::Vector3d::Zero();
    goal.control_mode = SE3Controller::ControlMode::kPosition;
    return goal;
  }

  const auto example_mode = static_cast<ExampleMode>(config_.simulation.example_mode);
  if (example_mode == ExampleMode::kSimple) {
    goal.state.position = config_.hover_goal.position;
    goal.state.velocity = velocity_mode ? config_.hover_goal.velocity : Eigen::Vector3d::Zero();
    if (goal.state.velocity.head<2>().norm() > 1e-6) {
      goal.forward = Eigen::Vector3d(goal.state.velocity.x(), goal.state.velocity.y(), 0.0).normalized();
    }
    return goal;
  }

  constexpr double kPi = 3.14159265358979323846;
  const double phase = 2.0 * kPi * circle.speed_hz * (context.sim_time - circle.wait_time);
  const double cosine = std::cos(phase);
  const double sine = std::sin(phase);

  goal.state.position = Eigen::Vector3d(circle.radius * cosine, circle.radius * sine, circle.height);
  goal.forward = Eigen::Vector3d(-sine, cosine, 0.0);

  if (velocity_mode) {
    const Eigen::Vector2d base_xy(config_.hover_goal.velocity.x(), config_.hover_goal.velocity.y());
    const Eigen::Matrix2d rotation =
        (Eigen::Matrix2d() << cosine, -sine, sine, cosine).finished();
    const Eigen::Vector2d rotated_xy = rotation * base_xy;
    const double vertical_velocity =
        config_.hover_goal.velocity.z() +
        circle.height_gain * (config_.hover_goal.position.z() - context.current_state.position.z());

    goal.state.velocity = Eigen::Vector3d(rotated_xy.x(), rotated_xy.y(), vertical_velocity);
    if (rotated_xy.norm() > 1e-6) {
      goal.forward = Eigen::Vector3d(rotated_xy.x(), rotated_xy.y(), 0.0).normalized();
    }
  }

  return goal;
}

CommandGoalProvider::CommandGoalProvider(
    const QuadrotorConfig& config,
    std::shared_ptr<CommandMailbox> command_mailbox)
    : command_mailbox_(std::move(command_mailbox)) {
  (void)config;
}

GoalReference CommandGoalProvider::Evaluate(const GoalContext& context) {
  const std::optional<VelocityCommand> command = command_mailbox_->ReadFresh();
  if (!yaw_initialized_) {
    desired_yaw_ = QuaternionToYaw(context.current_state.quaternion);
    last_sim_time_ = context.sim_time;
    yaw_initialized_ = true;
  }

  if (!command.has_value()) {
    last_sim_time_ = context.sim_time;
    GoalReference goal;
    goal.source = "ros2_hold";
    goal.control_mode = SE3Controller::ControlMode::kPosition;
    goal.state.position = context.current_state.position;
    goal.state.velocity = Eigen::Vector3d::Zero();
    goal.state.omega = Eigen::Vector3d::Zero();
    goal.state.quaternion = Eigen::Quaterniond::Identity();
    goal.forward = ForwardFromYaw(desired_yaw_);
    return goal;
  }

  const double dt = std::max(0.0, context.sim_time - last_sim_time_);
  desired_yaw_ += command->angular.z() * dt;
  last_sim_time_ = context.sim_time;

  GoalReference goal;
  goal.source = "ros2_cmd_vel";
  goal.control_mode = SE3Controller::ControlMode::kVelocity;
  goal.state.position = context.current_state.position;
  goal.state.velocity = command->linear;
  goal.state.omega = Eigen::Vector3d(0.0, 0.0, command->angular.z());
  goal.state.quaternion = Eigen::Quaterniond::Identity();
  goal.forward = ForwardFromYaw(desired_yaw_);
  return goal;
}

void CommandGoalProvider::Reset() {
  yaw_initialized_ = false;
  desired_yaw_ = 0.0;
  last_sim_time_ = 0.0;
}

}  // namespace quadrotor
