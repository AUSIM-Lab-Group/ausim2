// sim_mujoco_copy/converts/data/odom.cc
#include "./odom.h"
#include <chrono>

namespace sim::converts {

inline void SetHeader(std_msgs::msg::Header& header, const std::string& frame_id = "") {
  auto timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
  header.frame_id = frame_id;
  header.stamp.sec = timestamp / 1e9;
  header.stamp.nanosec = timestamp % static_cast<uint64_t>(1e9);
}

void Converts(nav_msgs::msg::Odometry& out, const sim::data::OdomData& in) {
  SetHeader(out.header, "odom");     // 父坐标系设为odom
  out.child_frame_id = "base_link";  // 子坐标系设为机器人基坐标系

  // 位置
  out.pose.pose.position.x = in.position.x;
  out.pose.pose.position.y = in.position.y;
  out.pose.pose.position.z = in.position.z;

  // 姿态
  out.pose.pose.orientation.w = in.orientation.w;
  out.pose.pose.orientation.x = in.orientation.x;
  out.pose.pose.orientation.y = in.orientation.y;
  out.pose.pose.orientation.z = in.orientation.z;

  // 线速度
  out.twist.twist.linear.x = in.linear_velocity.x;
  out.twist.twist.linear.y = in.linear_velocity.y;
  out.twist.twist.linear.z = in.linear_velocity.z;

  // 角速度
  out.twist.twist.angular.x = in.angular_velocity.x;
  out.twist.twist.angular.y = in.angular_velocity.y;
  out.twist.twist.angular.z = in.angular_velocity.z;
}

}  // namespace sim::converts