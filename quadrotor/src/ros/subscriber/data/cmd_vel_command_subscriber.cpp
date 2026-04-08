#include "ros/subscriber/data/cmd_vel_command_subscriber.hpp"

#if defined(QUADROTOR_HAS_ROS2)
#include <chrono>
#include <utility>

#include "converts/data/cmd_vel.hpp"

namespace quadrotor {

CmdVelCommandSubscriber::CmdVelCommandSubscriber(
    const std::shared_ptr<rclcpp::Node>& node,
    std::string topic_name,
    std::shared_ptr<CommandMailbox> command_mailbox)
    : subscription_(node->create_subscription<geometry_msgs::msg::Twist>(
          std::move(topic_name),
          10,
          [command_mailbox = std::move(command_mailbox)](
              const geometry_msgs::msg::Twist::SharedPtr msg) {
            if (!command_mailbox) {
              return;
            }
            command_mailbox->Write(converts::ToVelocityCommand(
                converts::ToCmdVelData(*msg),
                std::chrono::steady_clock::now()));
          })) {}

}  // namespace quadrotor
#endif
