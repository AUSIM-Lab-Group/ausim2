#include "converts/data/transform.hpp"

#include "converts/data/common.hpp"

namespace quadrotor::converts {

data::TransformData ToTransformData(
    const TelemetrySnapshot& snapshot,
    const std::string& frame_id,
    const std::string& child_frame_id) {
  data::TransformData out;
  out.header = BuildHeader(snapshot.sim_time, frame_id);
  out.child_frame_id = child_frame_id;
  out.translation = ToVector3(snapshot.state.position);
  out.rotation = ToQuaternion(snapshot.state.quaternion);
  return out;
}

#if defined(QUADROTOR_HAS_ROS2)
void Convert(geometry_msgs::msg::TransformStamped& out, const data::TransformData& in) {
  out.header.stamp = ToRosTime(in.header.stamp_seconds);
  out.header.frame_id = in.header.frame_id;
  out.child_frame_id = in.child_frame_id;
  Convert(out.transform.translation, in.translation);
  Convert(out.transform.rotation, in.rotation);
}

geometry_msgs::msg::TransformStamped ToRosMessage(const data::TransformData& in) {
  geometry_msgs::msg::TransformStamped out;
  Convert(out, in);
  return out;
}
#endif

}  // namespace quadrotor::converts
