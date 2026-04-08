#pragma once

// #include <sensor_msgs/msg/joint_command.hpp>
#include <aimdk_msgs/msg/joint_command_array.hpp>
// #include "joint/msg/joint_command.hpp"
#include "aima/sim/manager/subscriber/subscriber_base.h"

namespace aimrt::module::subscribe {

template <class T>
class LiftJointCommandSubscriber : public BaseSubscriber {
  using MessageType = T;

 public:
  LiftJointCommandSubscriber();
  ~LiftJointCommandSubscriber() override = default;

  bool Init() override;

 private:
  void Handle(const MessageType& msg);
};

REGISTER_SUBSCRIBER_TEMPLATE_IMPL(lift_joint_command, LiftJointCommandSubscriber<aimdk_msgs::msg::JointCommandArray>);
// REGISTER_SUBSCRIBER_TEMPLATE_IMPL(lift_joint_command_ros, LiftJointCommandSubscriber<sensor_msgs::msg::JointCommand>);

}  // namespace aimrt::module::subscribe