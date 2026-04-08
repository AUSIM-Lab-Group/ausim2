#include "converts/data/clock.hpp"

#include "converts/data/common.hpp"

namespace quadrotor::converts {

data::ClockData ToClockData(double stamp_seconds) {
  data::ClockData out;
  out.stamp_seconds = stamp_seconds;
  return out;
}

#if defined(QUADROTOR_HAS_ROS2)
void Convert(rosgraph_msgs::msg::Clock& out, const data::ClockData& in) {
  out.clock = ToRosTime(in.stamp_seconds);
}

rosgraph_msgs::msg::Clock ToRosMessage(const data::ClockData& in) {
  rosgraph_msgs::msg::Clock out;
  Convert(out, in);
  return out;
}
#endif

}  // namespace quadrotor::converts
