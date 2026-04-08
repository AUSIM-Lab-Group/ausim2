#pragma once

#include <aimdk_msgs/msg/joint_command_array.hpp>
#include <std_msgs/msg/string.hpp>
#include "aima/sim/manager/subscriber/subscriber_base.h"

namespace aimrt::module::subscribe {

class OptionCommandSubscriber : public BaseSubscriber {
  using MessageType = std_msgs::msg::String;

 public:
  OptionCommandSubscriber();
  ~OptionCommandSubscriber() override = default;

  bool Init() override;

 private:
  void Handle(const MessageType& msg);
};

REGISTER_SUBSCRIBER_IMPL(option_command, OptionCommandSubscriber);

}  // namespace aimrt::module::subscribe