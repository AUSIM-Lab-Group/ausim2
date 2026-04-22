#include "ros/publisher/semantic/robot_mode_publisher.hpp"

#include <utility>

#include "converts/ausim_msg/robot_mode.hpp"

namespace ausim {

RobotModePublisher::RobotModePublisher(const std::shared_ptr<rclcpp::Node>& node, std::string topic_name, VehicleIdentity identity)
    : publisher_(node->create_publisher<ausim_msg::msg::RobotMode>(std::move(topic_name), 10)), identity_(std::move(identity)) {}

void RobotModePublisher::Publish(const ipc::TelemetryPacket& packet) { publisher_->publish(converts::ToRosMessage(packet, identity_)); }

}  // namespace ausim
