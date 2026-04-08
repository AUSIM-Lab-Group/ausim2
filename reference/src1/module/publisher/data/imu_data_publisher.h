#pragma once

#include "aima/sim/converts/data/imu.h"
#include "aima/sim/manager/publisher/publisher_base.h"

namespace aimrt::module::publish {

template <class T>
class ImuDataPublisher : public BasePublisher {
  using MessageType = T;

 public:
  ImuDataPublisher();
  ~ImuDataPublisher() override = default;

  bool Init() override;
  void Publish() override;

 private:
  aimrt::channel::PublisherRef publisher_;
};

REGISTER_PUBLISHER_IMPL(imu_state_ros, ImuDataPublisher<sensor_msgs::msg::Imu>);
REGISTER_PUBLISHER_IMPL(imu_chest_state_ros, ImuDataPublisher<sensor_msgs::msg::Imu>);

}  // namespace aimrt::module::publish
