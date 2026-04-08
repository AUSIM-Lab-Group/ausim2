#pragma once

#include <memory>

#include "config/quadrotor_config.hpp"
#include "control/motor_mixer.hpp"
#include "runtime/goal_provider.hpp"

namespace quadrotor {

class VehicleRuntime {
 public:
  VehicleRuntime(
      const QuadrotorConfig& config,
      std::shared_ptr<CommandMailbox> command_mailbox);

  RuntimeOutput Step(const RuntimeInput& input, bool recompute_control, double control_dt);
  void Reset();

 private:
  QuadrotorConfig config_;
  SE3Controller controller_;
  MotorMixer mixer_;
  std::unique_ptr<GoalProvider> goal_provider_;
  ControlCommand cached_command_;
  GoalReference cached_goal_;
  Eigen::Vector4d cached_motor_speed_krpm_ = Eigen::Vector4d::Zero();
};

}  // namespace quadrotor
