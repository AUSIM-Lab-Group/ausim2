
#include "./joint_command_subscriber.h"
#include "./joint_command_tools.inc"
#include "aima/sim/converts/joint/joint_command.h"
#include "aima/sim/module/mujoco/interface.h"

namespace aimrt::module::subscribe {

template <class T>
JointCommandSubscriber<T>::JointCommandSubscriber() : BaseSubscriber() {}

template <class T>
bool JointCommandSubscriber<T>::Init() {
  if (config_.topic.empty()) {
    AIMRT_ERROR("Empty topic name.");
    return false;
  }

  Subscriber<MessageType>(std::bind(&JointCommandSubscriber::Handle, this, std::placeholders::_1));
  return true;
}

template <class T>
void JointCommandSubscriber<T>::Handle(const MessageType& msg) {
  // 从motion control API设置ArmJointCommand
  sim::data::JointCommands cmd;
  sim::converts::Converts(cmd, msg);

  if (IsEnable()) {
    sim::mj::SetJointCommand(cmd);  // SIM API
  }

  JOINT_TOOLS_PRINT_N_INFO(1, "Subscribe Joint Position: {}", sim::joint::tools::GetNamePosition(cmd));

  if (IsPrintable()) {
    AIMRT_INFO("Subscribe Joint Position: {}", sim::joint::tools::GetNamePosition(cmd));
  }
}

}  // namespace aimrt::module::subscribe