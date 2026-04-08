#pragma once

#include <memory>

#include "config/quadrotor_config.hpp"
#include "runtime/command_mailbox.hpp"
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
  CommandGoalProvider(
      const QuadrotorConfig& config,
      std::shared_ptr<CommandMailbox> command_mailbox);

  GoalReference Evaluate(const GoalContext& context) override;
 void Reset() override;

 private:
  std::shared_ptr<CommandMailbox> command_mailbox_;
  bool yaw_initialized_ = false;
  double desired_yaw_ = 0.0;
  double last_sim_time_ = 0.0;
};

}  // namespace quadrotor
