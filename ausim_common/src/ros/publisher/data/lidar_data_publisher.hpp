#pragma once

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "ipc/lidar_packet.hpp"

namespace ausim {

class LidarDataPublisher {
 public:
  LidarDataPublisher(const std::shared_ptr<rclcpp::Node>& node, std::string topic_name, std::string frame_id);

  void Publish(const ipc::LidarPacket& packet);

 private:
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  std::string frame_id_;
};

}  // namespace ausim
