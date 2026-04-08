#include "converts/data/odom.hpp"

#include "converts/data/common.hpp"

namespace quadrotor::converts {

data::OdomData ToOdomData(
    const TelemetrySnapshot& snapshot,
    const std::string& frame_id,
    const std::string& child_frame_id) {
  data::OdomData out;
  out.header = BuildHeader(snapshot.sim_time, frame_id);
  out.child_frame_id = child_frame_id;
  out.pose.position = ToVector3(snapshot.state.position);
  out.pose.orientation = ToQuaternion(snapshot.state.quaternion);
  out.twist.linear = ToVector3(snapshot.state.velocity);
  out.twist.angular = ToVector3(snapshot.state.omega);
  return out;
}

#if defined(QUADROTOR_HAS_ROS2)
void Convert(nav_msgs::msg::Odometry& out, const data::OdomData& in) {
  out.header.stamp = ToRosTime(in.header.stamp_seconds);
  out.header.frame_id = in.header.frame_id;
  out.child_frame_id = in.child_frame_id;
  Convert(out.pose.pose.position, in.pose.position);
  Convert(out.pose.pose.orientation, in.pose.orientation);
  Convert(out.twist.twist.linear, in.twist.linear);
  Convert(out.twist.twist.angular, in.twist.angular);
}

nav_msgs::msg::Odometry ToRosMessage(const data::OdomData& in) {
  nav_msgs::msg::Odometry out;
  Convert(out, in);
  return out;
}
#endif

}  // namespace quadrotor::converts
