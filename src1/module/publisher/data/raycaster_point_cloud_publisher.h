#pragma once

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include "aima/sim/manager/publisher/publisher_base.h"

namespace aimrt::module::publish {

template <class T>
class RaycasterPointCloudPublisher : public BasePublisher {
  using MessageType = T;

 public:
  RaycasterPointCloudPublisher();
  ~RaycasterPointCloudPublisher() override = default;

  bool Init() override;
  void Publish() override;

 private:
  aimrt::channel::PublisherRef publisher_;
};

// Switch to PointCloud2 so RViz2 can visualize directly
REGISTER_PUBLISHER_IMPL(raycaster_lidar_ros, RaycasterPointCloudPublisher<sensor_msgs::msg::PointCloud2>);

}  // namespace aimrt::module::publish
