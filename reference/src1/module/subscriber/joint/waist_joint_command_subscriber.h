#pragma once

// #include <sensor_msgs/msg/joint_command.hpp>
#include <aimdk_msgs/msg/joint_command_array.hpp>
#include "aima/sim/manager/subscriber/subscriber_base.h"

namespace aimrt::module::subscribe {

template <class T>
class WaistJointCommandSubscriber : public BaseSubscriber {
  using MessageType = T;

 public:
  WaistJointCommandSubscriber();
  ~WaistJointCommandSubscriber() override = default;

  bool Init() override;

 private:
  void Handle(const MessageType& msg);
};

REGISTER_SUBSCRIBER_TEMPLATE_IMPL(waist_joint_command, WaistJointCommandSubscriber<aimdk_msgs::msg::JointCommandArray>);
// REGISTER_SUBSCRIBER_TEMPLATE_IMPL(waist_joint_command_ros, WaistJointCommandSubscriber<sensor_msgs::msg::JointCommand>);

}  // namespace aimrt::module::subscribe