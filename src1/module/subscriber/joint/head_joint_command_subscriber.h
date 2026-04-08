#pragma once

// #include <sensor_msgs/msg/joint_command.hpp>
#include "aima/sim/manager/subscriber/subscriber_base.h"
// #include "aimdk/protocol/hal/joint/joint_channel.pb.h"
#include <aimdk_msgs/msg/joint_command_array.hpp>

namespace aimrt::module::subscribe {

template <class T>
class HeadJointCommandSubscriber : public BaseSubscriber {
  using MessageType = T;

 public:
  HeadJointCommandSubscriber();
  ~HeadJointCommandSubscriber() override = default;

  bool Init() override;

 private:
  void Handle(const MessageType& msg);
};

REGISTER_SUBSCRIBER_TEMPLATE_IMPL(head_joint_command, HeadJointCommandSubscriber<aimdk_msgs::msg::JointCommandArray>);
}  // namespace aimrt::module::subscribe