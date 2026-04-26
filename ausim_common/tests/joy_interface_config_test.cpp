#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

#include "config/quadrotor_config.hpp"

namespace {

void Expect(bool condition, const std::string& message) {
  if (!condition) {
    std::cerr << message << '\n';
    std::exit(1);
  }
}

}  // namespace

int main() {
  namespace fs = std::filesystem;

  const fs::path default_config_path = fs::temp_directory_path() / "joy_interface_default_config_test.yaml";
  std::ofstream default_output(default_config_path);
  default_output << R"yaml(
identity:
  namespace: /uav1
)yaml";
  default_output.close();

  const ausim::QuadrotorConfig default_config = ausim::LoadConfigFromYaml(default_config_path.string());
  Expect(default_config.interfaces.cmd_vel_topic == "/joy/cmd_vel", "expected default cmd_vel_topic to use /joy/cmd_vel");
  Expect(default_config.interfaces.joy_cmd_vel_topic == "/joy/cmd_vel", "expected default joy_cmd_vel_topic to use /joy/cmd_vel");
  fs::remove(default_config_path);

  const fs::path config_path = fs::temp_directory_path() / "joy_interface_config_test.yaml";
  std::ofstream output(config_path);
  output << R"yaml(
identity:
  namespace: /uav1

interfaces:
  cmd_vel_topic: cmd_vel
  joy_cmd_vel_topic: /joy/cmd_vel
  robot_mode_topic: teleop/mode
  robot_mode_structured_topic: teleop/mode_structured
  joy_action_services:
    - service: /joy/action1
      event: takeoff
    - service: /joy/action2
      event: land
    - service: /joy/action3
      event: reset
)yaml";
  output.close();

  const ausim::QuadrotorConfig config = ausim::LoadConfigFromYaml(config_path.string());
  Expect(config.interfaces.cmd_vel_topic == "cmd_vel", "expected cmd_vel_topic to stay unchanged");
  Expect(config.interfaces.joy_cmd_vel_topic == "/joy/cmd_vel", "expected joy_cmd_vel_topic to be loaded");
  Expect(config.interfaces.robot_mode_topic == "teleop/mode", "expected robot_mode_topic to be loaded");
  Expect(config.interfaces.robot_mode_structured_topic == "teleop/mode_structured", "expected robot_mode_structured_topic to be loaded");
  Expect(config.interfaces.joy_action_services.size() == 3, "expected three joy action services");
  Expect(config.interfaces.joy_action_services[0].service == "/joy/action1", "expected first joy action service path");
  Expect(config.interfaces.joy_action_services[0].event == "takeoff", "expected first joy action event");
  Expect(config.interfaces.joy_action_services[2].event == "reset", "expected reset to be preserved");

  fs::remove(config_path);
  return 0;
}
