#pragma once

#include <string>

#include "data/imu.hpp"
#include "runtime/runtime_types.hpp"

#if defined(QUADROTOR_HAS_ROS2)
#include <sensor_msgs/msg/imu.hpp>
#endif

namespace quadrotor::converts {

data::ImuData ToImuData(const TelemetrySnapshot& snapshot, const std::string& frame_id);

#if defined(QUADROTOR_HAS_ROS2)
void Convert(sensor_msgs::msg::Imu& out, const data::ImuData& in);
sensor_msgs::msg::Imu ToRosMessage(const data::ImuData& in);
#endif

}  // namespace quadrotor::converts
