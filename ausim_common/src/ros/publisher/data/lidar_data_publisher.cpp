#include "ros/publisher/data/lidar_data_publisher.hpp"

#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

namespace ausim {

LidarDataPublisher::LidarDataPublisher(const std::shared_ptr<rclcpp::Node>& node, std::string topic_name, std::string frame_id)
    : publisher_(node->create_publisher<sensor_msgs::msg::PointCloud2>(std::move(topic_name), rclcpp::QoS(10))),
      frame_id_(std::move(frame_id)) {}

void LidarDataPublisher::Publish(const ipc::LidarPacket& packet) {
  const int h = packet.h_ray_num;
  const int v = packet.v_ray_num;
  const int total = h * v;

  if (total <= 0 || total > ipc::LidarPacket::kMaxRays) {
    return;
  }

  // Compute XYZ for each valid ray and collect into a flat float buffer.
  // Ray layout in packet.data: index = h_index * v_ray_num + v_index
  //
  // Azimuth  (horizontal): h_index * (fov_h / h_ray_num) degrees, 0..fov_h
  // Elevation (vertical):  symmetric around 0, -fov_v/2 .. +fov_v/2 degrees
  const float fov_h_rad = packet.fov_h_deg * static_cast<float>(M_PI) / 180.0f;
  const float fov_v_rad = packet.fov_v_deg * static_cast<float>(M_PI) / 180.0f;
  const float azim_step = (h > 1) ? fov_h_rad / static_cast<float>(h) : 0.0f;
  const float elev_step = (v > 1) ? fov_v_rad / static_cast<float>(v - 1) : 0.0f;
  const float elev_start = -fov_v_rad / 2.0f;

  // Pre-allocate assuming all points hit; shrink at end.
  std::vector<float> xyz;
  xyz.reserve(total * 3);

  const float max_range = std::numeric_limits<float>::infinity();

  for (int hi = 0; hi < h; ++hi) {
    const float azim = static_cast<float>(hi) * azim_step;
    const float cos_azim = std::cos(azim);
    const float sin_azim = std::sin(azim);

    for (int vi = 0; vi < v; ++vi) {
      const float d = packet.data[hi * v + vi];
      if (!std::isfinite(d) || d <= 0.0f) {
        continue;
      }

      const float elev = elev_start + static_cast<float>(vi) * elev_step;
      const float cos_elev = std::cos(elev);
      xyz.push_back(d * cos_elev * cos_azim);
      xyz.push_back(d * cos_elev * sin_azim);
      xyz.push_back(d * std::sin(elev));
    }
  }

  const uint32_t n_points = static_cast<uint32_t>(xyz.size() / 3);

  sensor_msgs::msg::PointCloud2 msg;
  msg.header.stamp.sec = static_cast<int32_t>(packet.sim_time);
  msg.header.stamp.nanosec = static_cast<uint32_t>((packet.sim_time - msg.header.stamp.sec) * 1e9);
  msg.header.frame_id = frame_id_;
  msg.height = 1;
  msg.width = n_points;
  msg.is_dense = false;
  msg.is_bigendian = false;

  // Define x, y, z fields (float32).
  msg.fields.resize(3);
  for (int i = 0; i < 3; ++i) {
    msg.fields[i].name = std::string(1, static_cast<char>('x' + i));
    msg.fields[i].offset = static_cast<uint32_t>(i * sizeof(float));
    msg.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg.fields[i].count = 1;
  }

  msg.point_step = 3 * sizeof(float);
  msg.row_step = msg.point_step * n_points;
  msg.data.resize(msg.row_step);

  if (n_points > 0) {
    std::memcpy(msg.data.data(), xyz.data(), msg.data.size());
  }

  publisher_->publish(msg);
}

}  // namespace ausim
