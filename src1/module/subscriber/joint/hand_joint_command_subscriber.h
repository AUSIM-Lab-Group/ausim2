#pragma once

// #include <sensor_msgs/msg/joint_command.hpp>
#include "aima/sim/manager/subscriber/subscriber_base.h"
// #include "aimdk/protocol/hal/joint/joint_channel.pb.h"
#include <aimdk_msgs/msg/hand_command_array.hpp>

namespace aimrt::module::subscribe {

template <class T>
class HandJointCommandSubscriber : public BaseSubscriber {
  using MessageType = T;

 public:
  HandJointCommandSubscriber();
  ~HandJointCommandSubscriber() override = default;

  bool Init() override;

 private:
  void Handle(const MessageType& msg);
};

REGISTER_SUBSCRIBER_TEMPLATE_IMPL(hand_joint_command, HandJointCommandSubscriber<aimdk_msgs::msg::HandCommandArray>);
// REGISTER_SUBSCRIBER_TEMPLATE_IMPL(hand_joint_command_proto, HandJointCommandSubscriber<aimdk::protocol::JointCommandsChannel>);

}  // namespace aimrt::module::subscribe