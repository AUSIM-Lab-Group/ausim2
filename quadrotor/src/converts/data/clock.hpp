#pragma once

#include "data/clock.hpp"

#if defined(QUADROTOR_HAS_ROS2)
#include <rosgraph_msgs/msg/clock.hpp>
#endif

namespace quadrotor::converts {

data::ClockData ToClockData(double stamp_seconds);

#if defined(QUADROTOR_HAS_ROS2)
void Convert(rosgraph_msgs::msg::Clock& out, const data::ClockData& in);
rosgraph_msgs::msg::Clock ToRosMessage(const data::ClockData& in);
#endif

}  // namespace quadrotor::converts
