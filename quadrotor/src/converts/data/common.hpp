#pragma once

#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "data/common.hpp"

#if defined(QUADROTOR_HAS_ROS2)
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <rclcpp/time.hpp>
#endif

namespace quadrotor::converts {

data::Vector3 ToVector3(const Eigen::Vector3d& value);
data::Quaternion ToQuaternion(const Eigen::Quaterniond& value);
data::Header BuildHeader(double stamp_seconds, std::string frame_id);

#if defined(QUADROTOR_HAS_ROS2)
rclcpp::Time ToRosTime(double stamp_seconds);
void Convert(geometry_msgs::msg::Point& out, const data::Vector3& in);
void Convert(geometry_msgs::msg::Vector3& out, const data::Vector3& in);
void Convert(geometry_msgs::msg::Quaternion& out, const data::Quaternion& in);
#endif

}  // namespace quadrotor::converts
