#pragma once

#include <aimdk_msgs/msg/joint_command_array.hpp>
#include "aima/sim/manager/subscriber/subscriber_base.h"

namespace aimrt::module::subscribe {

template <class T>
class JointCommandSubscriber : public BaseSubscriber {
  using MessageType = T;

 public:
  JointCommandSubscriber();
  ~JointCommandSubscriber() override = default;

  bool Init() override;

 private:
  void Handle(const MessageType& msg);
};

REGISTER_SUBSCRIBER_TEMPLATE_IMPL(joint_command, JointCommandSubscriber<aimdk_msgs::msg::JointCommandArray>);
}  // namespace aimrt::module::subscribe