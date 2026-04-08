#pragma once

#include <time.h>
#include <cstdint>

#include <aimdk_msgs/msg/joint_state_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
// #include "joint/msg/joint_state.hpp"
#include "aima/sim/manager/publisher/publisher_base.h"

namespace aimrt::module::publish {

template <class T>
class WaistJointStatePublisher : public BasePublisher {
  using StateType = T;

 public:
  WaistJointStatePublisher();
  ~WaistJointStatePublisher() override = default;

  bool Init() override;
  void Publish() override;

 private:
  aimrt::channel::PublisherRef publisher_;
};

REGISTER_PUBLISHER_TEMPLATE_IMPL(waist_joint_state, WaistJointStatePublisher<aimdk_msgs::msg::JointStateArray>);
// REGISTER_PUBLISHER_TEMPLATE_IMPL(waist_joint_state_ros, WaistJointStatePublisher<sensor_msgs::msg::JointState>);

}  // namespace aimrt::module::publish