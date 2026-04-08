#pragma once

// #include <sensor_msgs/msg/joint_state.hpp>

#include <aimdk_msgs/msg/joint_state_array.hpp>
// #include "joint/msg/joint_state.hpp"
#include "aima/sim/manager/publisher/publisher_base.h"

namespace aimrt::module::publish {

template <class T>
class HeadJointStatePublisher : public BasePublisher {
  using StateType = T;

 public:
  HeadJointStatePublisher();
  ~HeadJointStatePublisher() override = default;

  bool Init() override;
  void Publish() override;

 private:
  aimrt::channel::PublisherRef publisher_;
};

REGISTER_PUBLISHER_TEMPLATE_IMPL(head_joint_state, HeadJointStatePublisher<aimdk_msgs::msg::JointStateArray>);
}  // namespace aimrt::module::publish