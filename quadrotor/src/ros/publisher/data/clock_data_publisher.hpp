#pragma once

#include <memory>
#include <string>

#if defined(QUADROTOR_HAS_ROS2)
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#endif

namespace quadrotor {

#if defined(QUADROTOR_HAS_ROS2)
class ClockDataPublisher {
 public:
  ClockDataPublisher(const std::shared_ptr<rclcpp::Node>& node, std::string topic_name);

  void Publish(double stamp_seconds) const;

 private:
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr publisher_;
};
#endif

}  // namespace quadrotor
