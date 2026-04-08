#pragma once

#include <grid_map_msgs/msg/grid_map.hpp>
#include "aima/sim/manager/publisher/publisher_base.h"

namespace aimrt::module::publish {

/**
 * @brief 发布 GridMap 消息的发布器
 * @details 从 RaycasterData 中获取 GridMap 数据并发布
 */
class RaycasterGridMapPublisher : public BasePublisher {
 public:
  RaycasterGridMapPublisher();
  ~RaycasterGridMapPublisher() override = default;

  bool Init() override;
  void Publish() override;

 private:
  aimrt::channel::PublisherRef publisher_;
};

// 注册 GridMap 发布器
REGISTER_PUBLISHER_IMPL(raycaster_grid_map_ros, RaycasterGridMapPublisher);

}  // namespace aimrt::module::publish
