#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include "aima/sim/data/raycaster.hpp"

namespace sim::converts {

/**
 * @brief 将 PointCloud2 转换为 PointCloud2（主要用于数据验证和准备）
 * @param out 输出数据
 * @param in 输入的 PointCloud2 消息
 */
void ConvertsPointCloud(sim::data::RaycasterData& out, const sensor_msgs::msg::PointCloud2& in);

/**
 * @brief 将 PointCloud2 转换为 GridMap
 * @param out 输出数据
 * @param in 输入的 PointCloud2 消息
 */
void ConvertsGridMap(sim::data::RaycasterData& out, const sensor_msgs::msg::PointCloud2& in);

}  // namespace sim::converts
