#pragma once

// #include <rclcpp/rclcpp.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

// #include <std_msgs/msg/float64_multi_array.hpp>

namespace sim::data {

/**
 * @brief Represents the data from a raycaster sensor (LiDAR).
 * @details Contains point cloud data with positions and intensities.
 */

struct Point3D {
  double x;
  double y;
  double z;
};

struct Quat {
  double w;
  double x;
  double y;
  double z;
};

struct RaycasterData {
  sensor_msgs::msg::PointCloud2 msg;         // PointCloud2 消息
  grid_map_msgs::msg::GridMap grid_map_msg;  // GridMap 消息
};

}  // namespace sim::data
