#pragma once

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "ausim_msg/msg/robot_mode.hpp"
#include "config/quadrotor_config.hpp"
#include "ros/publisher/i_telemetry_publisher.hpp"

namespace ausim {

class RobotModePublisher : public ITelemetryPublisher {
 public:
  RobotModePublisher(const std::shared_ptr<rclcpp::Node>& node, std::string topic_name, VehicleIdentity identity);

  void Publish(const ipc::TelemetryPacket& packet) override;

 private:
  rclcpp::Publisher<ausim_msg::msg::RobotMode>::SharedPtr publisher_;
  VehicleIdentity identity_;
};

}  // namespace ausim
