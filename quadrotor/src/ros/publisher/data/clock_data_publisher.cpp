#include "ros/publisher/data/clock_data_publisher.hpp"

#if defined(QUADROTOR_HAS_ROS2)
#include <utility>

#include "converts/data/clock.hpp"

namespace quadrotor {

ClockDataPublisher::ClockDataPublisher(
    const std::shared_ptr<rclcpp::Node>& node,
    std::string topic_name)
    : publisher_(node->create_publisher<rosgraph_msgs::msg::Clock>(std::move(topic_name), 10)) {}

void ClockDataPublisher::Publish(double stamp_seconds) const {
  publisher_->publish(converts::ToRosMessage(converts::ToClockData(stamp_seconds)));
}

}  // namespace quadrotor
#endif
