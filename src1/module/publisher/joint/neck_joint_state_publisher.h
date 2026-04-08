#pragma once

// #include <sensor_msgs/msg/joint_state.hpp>

#include <aimdk_msgs/msg/joint_state_array.hpp>
// #include "joint/msg/joint_state.hpp"
#include "aima/sim/manager/publisher/publisher_base.h"

namespace aimrt::module::publish {

template <class T>
class NeckJointStatePublisher : public BasePublisher {
  using StateType = T;

 public:
  NeckJointStatePublisher();
  ~NeckJointStatePublisher() override = default;

  bool Init() override;
  void Publish() override;

 private:
  aimrt::channel::PublisherRef publisher_;
};

REGISTER_PUBLISHER_TEMPLATE_IMPL(neck_joint_state, NeckJointStatePublisher<aimdk_msgs::msg::JointStateArray>);
// REGISTER_PUBLISHER_TEMPLATE_IMPL(Neck_joint_state_ros, NeckJointStatePublisher<sensor_msgs::msg::JointState>);
}  // namespace aimrt::module::publish