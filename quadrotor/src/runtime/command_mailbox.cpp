#include "runtime/command_mailbox.hpp"

#include <chrono>

namespace quadrotor {

CommandMailbox::CommandMailbox(double timeout_seconds) : timeout_seconds_(timeout_seconds) {}

void CommandMailbox::SetTimeoutSeconds(double timeout_seconds) {
  const std::lock_guard<std::mutex> lock(mutex_);
  timeout_seconds_ = timeout_seconds;
}

void CommandMailbox::Write(const VelocityCommand& command) {
  const std::lock_guard<std::mutex> lock(mutex_);
  latest_command_ = command;
}

std::optional<VelocityCommand> CommandMailbox::ReadFresh() const {
  const std::lock_guard<std::mutex> lock(mutex_);
  if (!latest_command_.has_value()) {
    return std::nullopt;
  }

  const auto age = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - latest_command_->received_time);
  if (age.count() > timeout_seconds_) {
    return std::nullopt;
  }
  return latest_command_;
}

}  // namespace quadrotor
