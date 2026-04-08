#pragma once

#include <string>

#include "data/odom.hpp"
#include "runtime/runtime_types.hpp"

#if defined(QUADROTOR_HAS_ROS2)
#include <nav_msgs/msg/odometry.hpp>
#endif

namespace quadrotor::converts {

data::OdomData ToOdomData(
    const TelemetrySnapshot& snapshot,
    const std::string& frame_id,
    const std::string& child_frame_id);

#if defined(QUADROTOR_HAS_ROS2)
void Convert(nav_msgs::msg::Odometry& out, const data::OdomData& in);
nav_msgs::msg::Odometry ToRosMessage(const data::OdomData& in);
#endif

}  // namespace quadrotor::converts
