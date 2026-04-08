#include "./raycaster.h"
#include <pcl/common/common.h>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <grid_map_core/GridMap.hpp>

namespace sim::converts {

void ConvertsPointCloud(sim::data::RaycasterData& out, const sensor_msgs::msg::PointCloud2& in) { out.msg = in; }

void ConvertsGridMap(sim::data::RaycasterData& out, const sensor_msgs::msg::PointCloud2& in) {
  // 1. 将 PointCloud2 转换为 PCL 点云
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(in, *cloud);

  // 检查点云是否为空
  if (cloud->empty()) {
    // 如果点云为空，创建空的 GridMap 消息
    out.grid_map_msg = grid_map_msgs::msg::GridMap();
    return;
  }

  // 2. 计算点云的边界（用于确定 GridMap 的几何）
  pcl::PointXYZ minBound, maxBound;
  pcl::getMinMax3D(*cloud, minBound, maxBound);

  // 检查边界是否有效（防止除零和无效范围）
  const double lengthX = maxBound.x - minBound.x;
  const double lengthY = maxBound.y - minBound.y;

  // 如果点云范围太小或无效，创建空的 GridMap
  if (lengthX < 0.01 || lengthY < 0.01 || std::isnan(lengthX) || std::isnan(lengthY) || std::isinf(lengthX) || std::isinf(lengthY)) {
    out.grid_map_msg = grid_map_msgs::msg::GridMap();
    return;
  }

  // 3. 设置 GridMap 参数
  const double resolution = 0.05;             // 5 cm 分辨率
  const std::string layerName = "elevation";  // 高程层名称

  // 限制最大 GridMap 尺寸，防止内存爆炸
  const double maxLength = 100.0;  // 最大 100m x 100m
  const double actualLengthX = std::min(lengthX, maxLength);
  const double actualLengthY = std::min(lengthY, maxLength);

  // 计算 GridMap 的尺寸和位置
  grid_map::Length length(actualLengthX, actualLengthY);
  grid_map::Position position((maxBound.x + minBound.x) / 2.0, (maxBound.y + minBound.y) / 2.0);

  // 4. 创建并初始化 GridMap
  grid_map::GridMap gridMap;
  try {
    gridMap.setGeometry(length, resolution, position);
    gridMap.setFrameId(in.header.frame_id.empty() ? "map" : in.header.frame_id);
    gridMap.add(layerName);
  } catch (const std::exception& e) {
    // 如果创建 GridMap 失败（可能因为太大），返回空消息
    out.grid_map_msg = grid_map_msgs::msg::GridMap();
    return;
  }

  // 5. 从点云填充 GridMap 的高程数据
  // 对于每个网格单元，找到该单元内所有点的最小 z 值（地面高度）
  grid_map::Matrix& elevationData = gridMap[layerName];

  // 遍历所有点，将点分配到对应的网格单元
  for (const auto& point : cloud->points) {
    // 获取点对应的网格索引
    grid_map::Index index;
    grid_map::Position pointPosition(point.x, point.y);

    if (gridMap.getIndex(pointPosition, index)) {
      // 如果该网格单元还没有值，或者当前点的 z 值更小（更接近地面），则更新
      if (!gridMap.isValid(index, layerName) || elevationData(index(0), index(1)) > point.z) {
        elevationData(index(0), index(1)) = point.z;
      }
    }
  }

  // 6. 设置时间戳
  gridMap.setTimestamp(in.header.stamp.sec + in.header.stamp.nanosec * 1e-9);

  // 7. 将 GridMap 转换为 ROS 消息
  try {
    auto gridMapMsgPtr = grid_map::GridMapRosConverter::toMessage(gridMap);
    if (gridMapMsgPtr) {
      out.grid_map_msg = *gridMapMsgPtr;
    } else {
      out.grid_map_msg = grid_map_msgs::msg::GridMap();
    }
  } catch (const std::exception& e) {
    // 如果转换失败，返回空消息
    out.grid_map_msg = grid_map_msgs::msg::GridMap();
  }
}

}  // namespace sim::converts
