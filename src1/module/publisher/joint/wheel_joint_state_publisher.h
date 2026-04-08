#pragma once

#include <sensor_msgs/msg/joint_state.hpp>
// #include "joint/msg/joint_state.hpp"
#include "aima/sim/manager/publisher/publisher_base.h"
#include "aimdk_msgs/msg/joint_state_array.hpp"

namespace aimrt::module::publish {

template <class T>
class WheelJointStatePublisher : public BasePublisher {
  using StateType = T;

 public:
  WheelJointStatePublisher();
  ~WheelJointStatePublisher() override = default;

  bool Init() override;
  void Publish() override;

 private:
  aimrt::channel::PublisherRef publisher_;
};

REGISTER_PUBLISHER_TEMPLATE_IMPL(wheel_joint_state, WheelJointStatePublisher<aimdk_msgs::msg::JointStateArray>);
// REGISTER_PUBLISHER_TEMPLATE_IMPL(wheel_joint_state_ros, WheelJointStatePublisher<sensor_msgs::msg::JointState>);

}  // namespace aimrt::module::publish