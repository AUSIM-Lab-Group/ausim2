#pragma once

// #include <sensor_msgs/msg/joint_state.hpp>

#include <aimdk_msgs/msg/joint_state_array.hpp>
// #include "joint/msg/joint_state.hpp"
#include "aima/sim/manager/publisher/publisher_base.h"

namespace aimrt::module::publish {

template <class T>
class ArmJointStatePublisher : public BasePublisher {
  using StateType = T;

 public:
  ArmJointStatePublisher();
  ~ArmJointStatePublisher() override = default;

  bool Init() override;
  void Publish() override;

 private:
  aimrt::channel::PublisherRef publisher_;
};

REGISTER_PUBLISHER_TEMPLATE_IMPL(arm_joint_state, ArmJointStatePublisher<aimdk_msgs::msg::JointStateArray>);
// REGISTER_PUBLISHER_TEMPLATE_IMPL(arm_joint_state_ros, ArmJointStatePublisher<sensor_msgs::msg::JointState>);
}  // namespace aimrt::module::publish