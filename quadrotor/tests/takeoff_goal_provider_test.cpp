#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>

#include "config/quadrotor_config.hpp"
#include "runtime/data_board_interface.hpp"
#include "runtime/goal_provider.hpp"
#include "runtime/runtime_types.hpp"

namespace {

void Expect(bool condition, const std::string& message) {
  if (!condition) {
    std::cerr << message << '\n';
    std::exit(1);
  }
}

quadrotor::CommandGoalProvider BuildProvider() {
  ausim::QuadrotorConfig config;
  config.ros2.command_timeout = 0.5;
  config.simulation.control_mode = static_cast<int>(quadrotor::SE3Controller::ControlMode::kVelocity);
  config.teleop_mode.initial_state = "on_ground";
  config.teleop_mode.actions.takeoff.height = 1.0;
  config.teleop_mode.states = {
      {"on_ground", "SAFE", false},
      {"hover", "MANUAL_READY", true},
  };
  config.teleop_mode.transitions = {
      {"on_ground", "hover", "takeoff", "", "", "takeoff", 0.0},
  };
  return quadrotor::CommandGoalProvider(config);
}

quadrotor::State BuildState() {
  quadrotor::State state;
  state.position = Eigen::Vector3d(0.0, 0.0, 0.01);
  state.quaternion = Eigen::Quaterniond::Identity();
  return state;
}

}  // namespace

int main() {
  quadrotor::ClearVelocityCommand();

  quadrotor::CommandGoalProvider provider = BuildProvider();
  const quadrotor::State current_state = BuildState();
  const quadrotor::GoalContext context{0.0, current_state};

  quadrotor::DiscreteCommand takeoff;
  takeoff.event_name = "takeoff";
  takeoff.kind = quadrotor::DiscreteCommandKind::kGenericEvent;
  takeoff.sequence = 1;

  const bool handled = provider.HandleDiscreteCommand(takeoff, context);
  Expect(handled, "takeoff command should be handled");

  quadrotor::VelocityCommand zero_command;
  zero_command.received_time = std::chrono::steady_clock::now();
  quadrotor::WriteVelocityCommand(zero_command);

  const quadrotor::GoalReference goal = provider.Evaluate(context);
  Expect(std::abs(goal.state.position.z() - 1.0) < 1e-9, "takeoff target height should remain 1.0 under fresh zero cmd_vel");
  Expect(goal.control_mode == quadrotor::SE3Controller::ControlMode::kPosition, "takeoff hover hold should use position control");

  quadrotor::ClearVelocityCommand();
  return 0;
}
