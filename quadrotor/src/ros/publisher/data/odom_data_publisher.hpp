#pragma once

#include <memory>
#include <string>

#include "runtime/runtime_types.hpp"

#if defined(QUADROTOR_HAS_ROS2)
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#endif

namespace quadrotor {

#if defined(QUADROTOR_HAS_ROS2)
class OdomDataPublisher {
 public:
  OdomDataPublisher(
      const std::shared_ptr<rclcpp::Node>& node,
      std::string topic_name,
      std::string frame_id,
      std::string child_frame_id);

  void Publish(const TelemetrySnapshot& snapshot) const;

 private:
  std::string frame_id_;
  std::string child_frame_id_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
};
#endif

}  // namespace quadrotor
