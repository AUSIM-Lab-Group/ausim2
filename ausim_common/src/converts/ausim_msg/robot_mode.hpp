#pragma once

#include "ausim_msg/msg/robot_mode.hpp"
#include "config/quadrotor_config.hpp"
#include "ipc/bridge_packets.hpp"

namespace ausim::converts {

void Convert(ausim_msg::msg::RobotMode& out, const ipc::TelemetryPacket& in, const VehicleIdentity& identity);
ausim_msg::msg::RobotMode ToRosMessage(const ipc::TelemetryPacket& in, const VehicleIdentity& identity);

}  // namespace ausim::converts
