#pragma once

#include <rosgraph_msgs/msg/clock.hpp>

#include "data/clock.hpp"

namespace quadrotor::converts {

data::ClockData ToClockData(double stamp_seconds);

void Convert(rosgraph_msgs::msg::Clock& out, const data::ClockData& in);
rosgraph_msgs::msg::Clock ToRosMessage(const data::ClockData& in);

}  // namespace quadrotor::converts
