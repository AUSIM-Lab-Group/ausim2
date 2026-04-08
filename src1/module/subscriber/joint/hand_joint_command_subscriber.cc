
#include "./hand_joint_command_subscriber.h"
#include "./joint_command_tools.inc"
#include "aima/sim/converts/joint/hand_command.h"
#include "aima/sim/module/mujoco/interface.h"

namespace aimrt::module::subscribe {

template <class T>
HandJointCommandSubscriber<T>::HandJointCommandSubscriber() : BaseSubscriber() {}

template <class T>
bool HandJointCommandSubscriber<T>::Init() {
  if (config_.topic.empty()) {
    AIMRT_ERROR("Empty topic name.");
    return false;
  }

  Subscriber<MessageType>(std::bind(&HandJointCommandSubscriber::Handle, this, std::placeholders::_1));
  return true;
}

template <class T>
void HandJointCommandSubscriber<T>::Handle(const MessageType& msg) {
  sim::data::JointCommands cmd;
  sim::converts::Converts(cmd, msg);

  if (IsEnable()) {
    sim::mj::SetHandJointCommand(cmd);
  }

  JOINT_TOOLS_PRINT_N_INFO(1, "Subscribe Hand Joint Position: {}", sim::joint::tools::GetNamePosition(cmd));

  if (IsPrintable()) {
    AIMRT_INFO("Subscribe Hand Joint Position: {}", sim::joint::tools::GetNamePosition(cmd));
  }
}

}  // namespace aimrt::module::subscribe