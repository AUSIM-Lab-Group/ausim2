#pragma once

// #include <sensor_msgs/msg/joint_command.hpp>
// #include "joint/msg/joint_command.hpp"
#include <aimdk_msgs/msg/joint_command_array.hpp>
#include "aima/sim/manager/subscriber/subscriber_base.h"

namespace aimrt::module::subscribe {

template <class T>
class WheelJointCommandSubscriber : public BaseSubscriber {
  using MessageType = T;

 public:
  WheelJointCommandSubscriber();
  ~WheelJointCommandSubscriber() override = default;

  bool Init() override;

 private:
  void Handle(const MessageType& msg);
};

REGISTER_SUBSCRIBER_TEMPLATE_IMPL(wheel_joint_command, WheelJointCommandSubscriber<aimdk_msgs::msg::JointCommandArray>);
// REGISTER_SUBSCRIBER_TEMPLATE_IMPL(wheel_joint_command_ros, WheelJointCommandSubscriber<sensor_msgs::msg::JointCommand>);

}  // namespace aimrt::module::subscribe