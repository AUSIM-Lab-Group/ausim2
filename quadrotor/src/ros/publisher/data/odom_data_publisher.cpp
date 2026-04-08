#include "ros/publisher/data/odom_data_publisher.hpp"

#if defined(QUADROTOR_HAS_ROS2)
#include <utility>

#include "converts/data/odom.hpp"

namespace quadrotor {

OdomDataPublisher::OdomDataPublisher(
    const std::shared_ptr<rclcpp::Node>& node,
    std::string topic_name,
    std::string frame_id,
    std::string child_frame_id)
    : frame_id_(std::move(frame_id)),
      child_frame_id_(std::move(child_frame_id)),
      publisher_(node->create_publisher<nav_msgs::msg::Odometry>(std::move(topic_name), 10)) {}

void OdomDataPublisher::Publish(const TelemetrySnapshot& snapshot) const {
  publisher_->publish(
      converts::ToRosMessage(converts::ToOdomData(snapshot, frame_id_, child_frame_id_)));
}

}  // namespace quadrotor
#endif
