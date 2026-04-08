#pragma once

#include <memory>

#include "config/quadrotor_config.hpp"
#include "runtime/command_mailbox.hpp"
#include "runtime/telemetry_cache.hpp"

namespace quadrotor {

class Ros2Bridge {
 public:
  virtual ~Ros2Bridge() = default;

  virtual void Start() = 0;
  virtual void Stop() = 0;
};

bool Ros2BridgeAvailable();
std::unique_ptr<Ros2Bridge> CreateRos2Bridge(
    const QuadrotorConfig& config,
    std::shared_ptr<CommandMailbox> command_mailbox,
    std::shared_ptr<TelemetryCache> telemetry_cache);

}  // namespace quadrotor
