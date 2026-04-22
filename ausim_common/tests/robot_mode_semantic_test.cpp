#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <string>

#include "converts/ausim_msg/robot_mode.hpp"
#include "ipc/bridge_packets.hpp"

namespace {

void Expect(bool condition, const std::string& message) {
  if (!condition) {
    std::cerr << message << '\n';
    std::exit(1);
  }
}

void ExpectEqual(std::uint32_t actual, std::uint32_t expected, const std::string& label) {
  if (actual != expected) {
    std::cerr << label << ": expected " << expected << ", got " << actual << '\n';
    std::exit(1);
  }
}

}  // namespace

int main() {
  ausim::VehicleIdentity identity;
  identity.vehicle_id = "uav1";
  identity.ros_namespace = "/uav1";

  ausim::ipc::TelemetryPacket packet;
  packet.sim_time = 12.3456789;
  packet.robot_top_level_state = ausim_msg::msg::RobotMode::TOP_STATE_MANUAL_ACTIVE;
  packet.robot_mode_accepts_motion = 1;
  packet.robot_mode_sub_state = {'t', 'a', 'k', 'e', 'o', 'f', 'f', '\0'};
  packet.last_discrete_command_sequence = 77;
  packet.last_discrete_command_status = ausim_msg::msg::RobotMode::ACK_SUCCESS;

  const ausim_msg::msg::RobotMode message = ausim::converts::ToRosMessage(packet, identity);

  Expect(message.vehicle_id == "uav1", "expected vehicle_id to be preserved");
  Expect(message.ros_namespace == "/uav1", "expected ros_namespace to be preserved");
  Expect(message.top_state == ausim_msg::msg::RobotMode::TOP_STATE_MANUAL_ACTIVE, "expected top_state mapping");
  Expect(message.sub_state == "takeoff", "expected sub_state mapping");
  Expect(message.accepts_motion, "expected accepts_motion mapping");
  Expect(message.last_discrete_command_sequence == 77, "expected discrete command sequence mapping");
  Expect(message.last_discrete_command_status == ausim_msg::msg::RobotMode::ACK_SUCCESS, "expected ack status mapping");
  ExpectEqual(message.stamp.sec, 12, "expected seconds component");
  ExpectEqual(message.stamp.nanosec, 345678900u, "expected nanoseconds component");
  return 0;
}
