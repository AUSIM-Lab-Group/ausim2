#pragma once

#include <nav_msgs/msg/odometry.hpp>
#include "aima/sim/converts/data/odom.h"  // 后续需补充Odom转换函数
#include "aima/sim/manager/publisher/publisher_base.h"

namespace aimrt::module::publish {

template <class T>
class OdomDataPublisher : public BasePublisher {
  using MessageType = T;

 public:
  OdomDataPublisher();
  ~OdomDataPublisher() override = default;

  bool Init() override;
  void Publish() override;

 private:
  aimrt::channel::PublisherRef publisher_;
};

REGISTER_PUBLISHER_IMPL(odom_state_ros, OdomDataPublisher<nav_msgs::msg::Odometry>);

}  // namespace aimrt::module::publish