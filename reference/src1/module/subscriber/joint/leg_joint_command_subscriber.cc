
#include "./leg_joint_command_subscriber.h"
#include "./joint_command_tools.inc"
#include "aima/sim/converts/converts.h"
#include "aima/sim/module/mujoco/interface.h"

namespace aimrt::module::subscribe {

template <class T>
LegJointCommandSubscriber<T>::LegJointCommandSubscriber() : BaseSubscriber() {}

template <class T>
bool LegJointCommandSubscriber<T>::Init() {
  if (config_.topic.empty()) {
    AIMRT_ERROR("Empty topic name.");
    return false;
  }

  Subscriber<MessageType>(std::bind(&LegJointCommandSubscriber::Handle, this, std::placeholders::_1));
  return true;
}

template <class T>
void LegJointCommandSubscriber<T>::Handle(const MessageType& msg) {
  // 从motion control API设置ArmJointCommand
  sim::data::JointCommands cmd;
  sim::converts::Converts(cmd, msg);

  if (IsEnable()) {
    // sim::api::SetLegJointCommand(cmd);  // SIM API
    sim::mj::SetLegJointCommand(cmd);  // SIM API
  }

  JOINT_TOOLS_PRINT_N_INFO(1, "Subscribe Leg Joint Position: {}", sim::joint::tools::GetNamePosition(cmd));

  if (IsPrintable()) {
    AIMRT_INFO("cmd size: {}", cmd.joints.size());
    AIMRT_INFO("Subscribe Leg Joint Position: {}", sim::joint::tools::GetNamePosition(cmd));
  }
}

}  // namespace aimrt::module::subscribe