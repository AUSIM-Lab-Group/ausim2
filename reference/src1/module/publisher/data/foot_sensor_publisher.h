#pragma once

#include "aima/sim/converts/data/imu.h"
#include "aima/sim/manager/publisher/publisher_base.h"

namespace aimrt::module::publish {

template <class T>
class ImuFootDataPublisher : public BasePublisher {
  using MessageType = T;

 public:
  ImuFootDataPublisher();
  ~ImuFootDataPublisher() override = default;

  bool Init() override;
  void Publish() override;

 private:
  aimrt::channel::PublisherRef publisher_;
};

REGISTER_PUBLISHER_TEMPLATE_IMPL(foot_sensor_ros, ImuFootDataPublisher<std_msgs::msg::Float64MultiArray>);

}  // namespace aimrt::module::publish