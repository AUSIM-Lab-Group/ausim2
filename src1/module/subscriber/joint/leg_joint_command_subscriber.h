#pragma once

// #include <sensor_msgs/msg/joint_command.hpp>

#include <aimdk_msgs/msg/joint_command_array.hpp>
// #include "joint/msg/joint_command.hpp"
#include "aima/sim/manager/subscriber/subscriber_base.h"

namespace aimrt::module::subscribe {

template <class T>
class LegJointCommandSubscriber : public BaseSubscriber {
  using MessageType = T;

 public:
  LegJointCommandSubscriber();
  ~LegJointCommandSubscriber() override = default;

  bool Init() override;

 private:
  void Handle(const MessageType& msg);
};

REGISTER_SUBSCRIBER_TEMPLATE_IMPL(leg_joint_command, LegJointCommandSubscriber<aimdk_msgs::msg::JointCommandArray>);
// REGISTER_SUBSCRIBER_TEMPLATE_IMPL(leg_joint_command_ros, LegJointCommandSubscriber<sensor_msgs::msg::JointCommand>);

}  // namespace aimrt::module::subscribe