#include "./raycaster_point_cloud_publisher.h"
#include "aima/sim/converts/data/raycaster.h"
#include "aima/sim/data/raycaster.hpp"
#include "aima/sim/module/mujoco/interface.h"

namespace aimrt::module::publish {

template <class T>
RaycasterPointCloudPublisher<T>::RaycasterPointCloudPublisher() : BasePublisher() {}

template <class T>
bool RaycasterPointCloudPublisher<T>::Init() {
  if (config_.frequency <= 0) {
    AIMRT_ERROR("Invalid frequency: {}", config_.frequency);
    return false;
  }

  if (config_.topic.empty()) {
    AIMRT_ERROR("Empty topic name.");
    return false;
  }

  publisher_ = aimrt::helper::GetPublisher<MessageType>(core_, config_.topic);
  if (!publisher_) {
    AIMRT_ERROR("Failed to get publisher: {}", config_.topic.c_str());
    return false;
  }
  return true;
}

template <class T>
void RaycasterPointCloudPublisher<T>::Publish() {
  if (!publisher_) {
    return;
  }
  std::string raycaster_name = config_.topic;

  const std::string raycaster_frame_type = "raycaster_lidar";

  // 从 sim 获取 raycaster 数据
  sim::data::RaycasterData raycaster_data;
  sim::mj::GetRaycasterData(raycaster_data, raycaster_frame_type);

  sim::converts::ConvertsPointCloud(raycaster_data, raycaster_data.msg);

  // 发布 PointCloud2 消息
  if (IsEnable()) {
    aimrt::channel::Publish(publisher_, raycaster_data.msg);
  }
}

REGISTER_PUBLISHER_TEMPLATE_INSTANTIATION(RaycasterPointCloudPublisher<sensor_msgs::msg::PointCloud2>);

}  // namespace aimrt::module::publish
