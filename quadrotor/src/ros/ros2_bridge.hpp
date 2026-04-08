#pragma once

#include "config/quadrotor_config.hpp"

namespace quadrotor {

int RunRosBridgeProcess(
    const QuadrotorConfig& config,
    int telemetry_fd,
    int command_fd,
    int image_fd);

}  // namespace quadrotor
