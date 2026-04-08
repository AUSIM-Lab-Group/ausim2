#include "ros/publisher/data/transform_data_publisher.hpp"

#if defined(QUADROTOR_HAS_ROS2)
#include <utility>

#include "converts/data/transform.hpp"

namespace quadrotor {

TransformDataPublisher::TransformDataPublisher(
    const std::shared_ptr<rclcpp::Node>& node,
    std::string frame_id,
    std::string child_frame_id)
    : frame_id_(std::move(frame_id)),
      child_frame_id_(std::move(child_frame_id)),
      broadcaster_(std::make_unique<tf2_ros::TransformBroadcaster>(node)) {}

void TransformDataPublisher::Publish(const TelemetrySnapshot& snapshot) const {
  broadcaster_->sendTransform(
      converts::ToRosMessage(converts::ToTransformData(snapshot, frame_id_, child_frame_id_)));
}

}  // namespace quadrotor
#endif
