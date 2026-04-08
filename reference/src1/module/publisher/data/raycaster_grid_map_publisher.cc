#include "./raycaster_grid_map_publisher.h"
#include "aima/sim/converts/data/raycaster.h"
#include "aima/sim/data/raycaster.hpp"
#include "aima/sim/module/mujoco/interface.h"

namespace aimrt::module::publish {

RaycasterGridMapPublisher::RaycasterGridMapPublisher() : BasePublisher() {}

bool RaycasterGridMapPublisher::Init() {
  if (config_.frequency <= 0) {
    AIMRT_ERROR("Invalid frequency: {}", config_.frequency);
    return false;
  }

  if (config_.topic.empty()) {
    AIMRT_ERROR("Empty topic name.");
    return false;
  }

  // 注册 GridMap 消息类型
  publisher_ = aimrt::helper::GetPublisher<grid_map_msgs::msg::GridMap>(core_, config_.topic);
  if (!publisher_) {
    AIMRT_ERROR("Failed to get publisher: {}", config_.topic.c_str());
    return false;
  }
  return true;
}

void RaycasterGridMapPublisher::Publish() {
  if (!publisher_) {
    return;
  }

  const std::string raycaster_frame_type = "raycaster_lidar";

  // 从 sim 获取 raycaster 数据
  sim::data::RaycasterData raycaster_data;
  sim::mj::GetRaycasterData(raycaster_data, raycaster_frame_type);

  sim::converts::ConvertsGridMap(raycaster_data, raycaster_data.msg);

  // 发布 GridMap 消息
  if (IsEnable()) {
    aimrt::channel::Publish(publisher_, raycaster_data.grid_map_msg);
  }

  if (IsPrintable()) {
    AIMRT_INFO("Publish GridMap: layers={}, frame_id={}", raycaster_data.grid_map_msg.layers.size(), raycaster_data.grid_map_msg.header.frame_id);
  }
}

}  // namespace aimrt::module::publish
