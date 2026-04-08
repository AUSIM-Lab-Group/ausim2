#pragma once

#include "config/quadrotor_config.hpp"
#include "runtime/runtime_types.hpp"

namespace quadrotor {

struct GoalContext {
  double sim_time = 0.0;
  const State& current_state;
};

class GoalProvider {
 public:
  virtual ~GoalProvider() = default;

  virtual GoalReference Evaluate(const GoalContext& context) = 0;
  virtual void Reset() {}
};

class DemoGoalProvider : public GoalProvider {
 public:
  explicit DemoGoalProvider(const QuadrotorConfig& config);

  GoalReference Evaluate(const GoalContext& context) override;

 private:
  const QuadrotorConfig& config_;
};

class CommandGoalProvider : public GoalProvider {
 public:
  explicit CommandGoalProvider(const QuadrotorConfig& config);

  GoalReference Evaluate(const GoalContext& context) override;
  void Reset() override;

 private:
  double command_timeout_seconds_ = 0.5;
  SE3Controller::ControlMode control_mode_ = SE3Controller::ControlMode::kVelocity;
  bool initialized_ = false;
  Eigen::Vector3d spawn_position_ = Eigen::Vector3d::Zero();
  double desired_yaw_ = 0.0;
  double last_sim_time_ = 0.0;
};

}  // namespace quadrotor
