#pragma once

#include <aimdk_msgs/msg/joint_state_array.hpp>
#include "aima/sim/manager/publisher/publisher_base.h"

namespace aimrt::module::publish {

template <class T>
class JointStatePublisher : public BasePublisher {
  using StateType = T;

 public:
  JointStatePublisher();
  ~JointStatePublisher() override = default;

  bool Init() override;
  void Publish() override;

 private:
  aimrt::channel::PublisherRef publisher_;
};

REGISTER_PUBLISHER_TEMPLATE_IMPL(joint_state, JointStatePublisher<aimdk_msgs::msg::JointStateArray>);
}  // namespace aimrt::module::publish