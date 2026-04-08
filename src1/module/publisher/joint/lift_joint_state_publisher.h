#pragma once

#include <aimdk_msgs/msg/joint_state_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
// #include "joint/msg/joint_state.hpp"
#include "aima/sim/manager/publisher/publisher_base.h"

namespace aimrt::module::publish {

template <class T>
class LiftJointStatePublisher : public BasePublisher {
  using StateType = T;

 public:
  LiftJointStatePublisher();
  ~LiftJointStatePublisher() override = default;

  bool Init() override;
  void Publish() override;

 private:
  aimrt::channel::PublisherRef publisher_;
};

REGISTER_PUBLISHER_TEMPLATE_IMPL(lift_joint_state, LiftJointStatePublisher<aimdk_msgs::msg::JointStateArray>);
// REGISTER_PUBLISHER_TEMPLATE_IMPL(lift_joint_state_ros, LiftJointStatePublisher<sensor_msgs::msg::JointState>);

}  // namespace aimrt::module::publish