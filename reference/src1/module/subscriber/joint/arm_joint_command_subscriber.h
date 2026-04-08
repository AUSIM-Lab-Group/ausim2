#pragma once

// #include <sensor_msgs/msg/joint_command.hpp>
#include "aima/sim/manager/subscriber/subscriber_base.h"
// #include "aimdk/protocol/hal/joint/joint_channel.pb.h"
#include <aimdk_msgs/msg/joint_command_array.hpp>

namespace aimrt::module::subscribe {

template <class T>
class ArmJointCommandSubscriber : public BaseSubscriber {
  using MessageType = T;

 public:
  ArmJointCommandSubscriber();
  ~ArmJointCommandSubscriber() override = default;

  bool Init() override;

 private:
  void Handle(const MessageType& msg);
};

REGISTER_SUBSCRIBER_TEMPLATE_IMPL(arm_joint_command, ArmJointCommandSubscriber<aimdk_msgs::msg::JointCommandArray>);
// REGISTER_SUBSCRIBER_TEMPLATE_IMPL(arm_joint_command_proto, ArmJointCommandSubscriber<aimdk::protocol::JointCommandsChannel>);

}  // namespace aimrt::module::subscribe