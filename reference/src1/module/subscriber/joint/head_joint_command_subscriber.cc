
#include "./head_joint_command_subscriber.h"
#include "./joint_command_tools.inc"
#include "aima/sim/converts/joint/joint_command.h"
#include "aima/sim/module/mujoco/interface.h"

namespace aimrt::module::subscribe {

template <class T>
HeadJointCommandSubscriber<T>::HeadJointCommandSubscriber() : BaseSubscriber() {}

template <class T>
bool HeadJointCommandSubscriber<T>::Init() {
  if (config_.topic.empty()) {
    AIMRT_ERROR("Empty topic name.");
    return false;
  }

  Subscriber<MessageType>(std::bind(&HeadJointCommandSubscriber::Handle, this, std::placeholders::_1));
  return true;
}

template <class T>
void HeadJointCommandSubscriber<T>::Handle(const MessageType& msg) {
  // 从motion control API设置HeadJointCommand
  sim::data::JointCommands cmd;
  sim::converts::Converts(cmd, msg);

  if (IsEnable()) {
    sim::mj::SetHeadJointCommand(cmd);  // SIM API
  }

  JOINT_TOOLS_PRINT_N_INFO(1, "Subscribe Head Joint Position: {}", sim::joint::tools::GetNamePosition(cmd));

  if (IsPrintable()) {
    AIMRT_INFO("Subscribe Head Joint Position: {}", sim::joint::tools::GetNamePosition(cmd));
  }
}

}  // namespace aimrt::module::subscribe