#pragma once

#include <memory>
#include <string>

#include "runtime/runtime_types.hpp"

#if defined(QUADROTOR_HAS_ROS2)
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#endif

namespace quadrotor {

#if defined(QUADROTOR_HAS_ROS2)
class TransformDataPublisher {
 public:
  TransformDataPublisher(
      const std::shared_ptr<rclcpp::Node>& node,
      std::string frame_id,
      std::string child_frame_id);

  void Publish(const TelemetrySnapshot& snapshot) const;

 private:
  std::string frame_id_;
  std::string child_frame_id_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
};
#endif

}  // namespace quadrotor
