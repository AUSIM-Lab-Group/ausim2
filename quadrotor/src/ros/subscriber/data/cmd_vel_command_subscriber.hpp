#pragma once

#include <memory>
#include <string>

#include "runtime/command_mailbox.hpp"

#if defined(QUADROTOR_HAS_ROS2)
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#endif

namespace quadrotor {

#if defined(QUADROTOR_HAS_ROS2)
class CmdVelCommandSubscriber {
 public:
  CmdVelCommandSubscriber(
      const std::shared_ptr<rclcpp::Node>& node,
      std::string topic_name,
      std::shared_ptr<CommandMailbox> command_mailbox);

 private:
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
};
#endif

}  // namespace quadrotor
