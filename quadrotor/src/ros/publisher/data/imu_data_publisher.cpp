#include "ros/publisher/data/imu_data_publisher.hpp"

#if defined(QUADROTOR_HAS_ROS2)
#include <utility>

#include "converts/data/imu.hpp"

namespace quadrotor {

ImuDataPublisher::ImuDataPublisher(
    const std::shared_ptr<rclcpp::Node>& node,
    std::string topic_name,
    std::string frame_id)
    : frame_id_(std::move(frame_id)),
      publisher_(node->create_publisher<sensor_msgs::msg::Imu>(std::move(topic_name), 10)) {}

void ImuDataPublisher::Publish(const TelemetrySnapshot& snapshot) const {
  publisher_->publish(converts::ToRosMessage(converts::ToImuData(snapshot, frame_id_)));
}

}  // namespace quadrotor
#endif
