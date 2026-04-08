#include "converts/data/imu.hpp"

#include "converts/data/common.hpp"

namespace quadrotor::converts {

data::ImuData ToImuData(const TelemetrySnapshot& snapshot, const std::string& frame_id) {
  data::ImuData out;
  out.header = BuildHeader(snapshot.sim_time, frame_id);
  out.orientation = ToQuaternion(snapshot.imu.orientation);
  out.angular_velocity = ToVector3(snapshot.imu.angular_velocity);
  out.linear_acceleration = ToVector3(snapshot.imu.linear_acceleration);
  out.has_linear_acceleration = snapshot.imu.has_linear_acceleration;
  return out;
}

#if defined(QUADROTOR_HAS_ROS2)
void Convert(sensor_msgs::msg::Imu& out, const data::ImuData& in) {
  out.header.stamp = ToRosTime(in.header.stamp_seconds);
  out.header.frame_id = in.header.frame_id;
  Convert(out.orientation, in.orientation);
  Convert(out.angular_velocity, in.angular_velocity);
  if (in.has_linear_acceleration) {
    Convert(out.linear_acceleration, in.linear_acceleration);
  }
}

sensor_msgs::msg::Imu ToRosMessage(const data::ImuData& in) {
  sensor_msgs::msg::Imu out;
  Convert(out, in);
  return out;
}
#endif

}  // namespace quadrotor::converts
