#include "./imu.h"
#include <chrono>

namespace sim::converts {

inline void SetHeader(std_msgs::msg::Header& header, const std::string& frame_id = "") {
  auto timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
  header.frame_id = frame_id;
  header.stamp.sec = timestamp / 1e9;
  header.stamp.nanosec = timestamp % static_cast<uint64_t>(1e9);
}

void Converts(sim::data::ImuData& out, const sensor_msgs::msg::Imu& in) {
  out.orientation = {in.orientation.w, in.orientation.x, in.orientation.y, in.orientation.z};
  out.angular_velocity = {in.angular_velocity.x, in.angular_velocity.y, in.angular_velocity.z};
  out.linear_acceleration = {in.linear_acceleration.x, in.linear_acceleration.y, in.linear_acceleration.z};
}

void Converts(sensor_msgs::msg::Imu& out, const sim::data::ImuData& in) {
  SetHeader(out.header);
  out.orientation.w = in.orientation.w;
  out.orientation.x = in.orientation.x;
  out.orientation.y = in.orientation.y;
  out.orientation.z = in.orientation.z;
  out.angular_velocity.x = in.angular_velocity.x;
  out.angular_velocity.y = in.angular_velocity.y;
  out.angular_velocity.z = in.angular_velocity.z;
  out.linear_acceleration.x = in.linear_acceleration.x;
  out.linear_acceleration.y = in.linear_acceleration.y;
  out.linear_acceleration.z = in.linear_acceleration.z;
}

void Converts(std_msgs::msg::Float64MultiArray& out, const std::vector<std::vector<double>>& in) {
  std::vector<double> vec1D;
  for (auto& vec : in) {
    for (auto& val : vec) {
      vec1D.push_back(val);
    }
  }
  out.data = vec1D;
}

void Converts(tf2_msgs::msg::TFMessage& out, const sim::data::ImuData& in) {
  tf2_msgs::msg::TFMessage tf_msg;
  geometry_msgs::msg::TransformStamped transform_stamped;
  // auto timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
  // transform_stamped.header.stamp.sec = timestamp / 1e9;
  // transform_stamped.header.stamp.nanosec = timestamp % static_cast<uint64_t>(1e9);
  // transform_stamped.header.frame_id = "odom";
  SetHeader(transform_stamped.header, "odom");
  transform_stamped.child_frame_id = "base_link";
  transform_stamped.transform.translation.x = in.linear_position.x;
  transform_stamped.transform.translation.y = in.linear_position.y;
  transform_stamped.transform.translation.z = 0.0;
  transform_stamped.transform.rotation.x = in.orientation.x;
  transform_stamped.transform.rotation.y = in.orientation.y;
  transform_stamped.transform.rotation.z = in.orientation.z;
  transform_stamped.transform.rotation.w = in.orientation.w;
  tf_msg.transforms.push_back(transform_stamped);
  out = tf_msg;
}

}  // namespace sim::converts
