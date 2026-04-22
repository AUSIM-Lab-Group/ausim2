#include "converts/ausim_msg/robot_mode.hpp"

#include <algorithm>
#include <cstdint>

#include "converts/data/common.hpp"

namespace ausim::converts {
namespace {

std::string FromFixedString(const std::array<char, ipc::kRobotModeNameCapacity>& buffer) {
  const auto nul = std::find(buffer.begin(), buffer.end(), '\0');
  return std::string(buffer.data(), static_cast<std::size_t>(std::distance(buffer.begin(), nul)));
}

}  // namespace

void Convert(ausim_msg::msg::RobotMode& out, const ipc::TelemetryPacket& in, const VehicleIdentity& identity) {
  const std::int64_t stamp_nanoseconds = ToRosTime(in.sim_time).nanoseconds();
  out.stamp.sec = static_cast<std::int32_t>(stamp_nanoseconds / 1'000'000'000LL);
  out.stamp.nanosec = static_cast<std::uint32_t>(stamp_nanoseconds % 1'000'000'000LL);
  out.vehicle_id = identity.vehicle_id;
  out.ros_namespace = identity.ros_namespace;
  out.top_state = in.robot_top_level_state;
  out.sub_state = FromFixedString(in.robot_mode_sub_state);
  out.accepts_motion = in.robot_mode_accepts_motion != 0;
  out.last_discrete_command_sequence = in.last_discrete_command_sequence;
  out.last_discrete_command_status = in.last_discrete_command_status;
}

ausim_msg::msg::RobotMode ToRosMessage(const ipc::TelemetryPacket& in, const VehicleIdentity& identity) {
  ausim_msg::msg::RobotMode out;
  Convert(out, in, identity);
  return out;
}

}  // namespace ausim::converts
