#pragma once

#include <mutex>
#include <optional>

#include "runtime/runtime_types.hpp"

namespace quadrotor {

class CommandMailbox {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  explicit CommandMailbox(double timeout_seconds = 0.5);

  void SetTimeoutSeconds(double timeout_seconds);
  void Write(const VelocityCommand& command);
  std::optional<VelocityCommand> ReadFresh() const;

 private:
  mutable std::mutex mutex_;
  std::optional<VelocityCommand> latest_command_;
  double timeout_seconds_ = 0.5;
};

}  // namespace quadrotor
