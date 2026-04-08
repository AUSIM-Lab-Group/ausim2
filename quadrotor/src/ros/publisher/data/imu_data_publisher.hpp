#pragma once

#include <memory>
#include <string>

#include "runtime/runtime_types.hpp"

#if defined(QUADROTOR_HAS_ROS2)
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#endif

namespace quadrotor {

#if defined(QUADROTOR_HAS_ROS2)
class ImuDataPublisher {
 public:
  ImuDataPublisher(
      const std::shared_ptr<rclcpp::Node>& node,
      std::string topic_name,
      std::string frame_id);

  void Publish(const TelemetrySnapshot& snapshot) const;

 private:
  std::string frame_id_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
};
#endif

}  // namespace quadrotor
