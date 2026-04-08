#pragma once

#include <chrono>
#include <string>

#include "data/clock.hpp"
#include "data/cmd_vel.hpp"
#include "data/imu.hpp"
#include "data/odom.hpp"
#include "data/transform.hpp"
#include "ipc/bridge_packets.hpp"
#include "runtime/runtime_types.hpp"

namespace quadrotor::converts {

ipc::VelocityCommandPacket ToVelocityCommandPacket(const data::CmdVelData& message);
VelocityCommand ToVelocityCommand(
    const ipc::VelocityCommandPacket& packet,
    std::chrono::steady_clock::time_point received_time);

ipc::TelemetryPacket ToTelemetryPacket(const TelemetrySnapshot& snapshot);

data::OdomData ToOdomData(
    const ipc::TelemetryPacket& packet,
    const std::string& frame_id,
    const std::string& child_frame_id);
data::ImuData ToImuData(const ipc::TelemetryPacket& packet, const std::string& frame_id);
data::TransformData ToTransformData(
    const ipc::TelemetryPacket& packet,
    const std::string& frame_id,
    const std::string& child_frame_id);
data::ClockData ToClockData(const ipc::TelemetryPacket& packet);

}  // namespace quadrotor::converts
