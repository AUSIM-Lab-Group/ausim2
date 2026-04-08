
#include "./waist_joint_command_subscriber.h"
#include "./joint_command_tools.inc"
#include "aima/sim/converts/converts.h"
#include "aima/sim/module/mujoco/interface.h"

namespace aimrt::module::subscribe {

template <class T>
WaistJointCommandSubscriber<T>::WaistJointCommandSubscriber() : BaseSubscriber() {}

template <class T>
bool WaistJointCommandSubscriber<T>::Init() {
  if (config_.topic.empty()) {
    AIMRT_ERROR("Empty topic name.");
    return false;
  }

  Subscriber<MessageType>(std::bind(&WaistJointCommandSubscriber::Handle, this, std::placeholders::_1));
  return true;
}

template <class T>
void WaistJointCommandSubscriber<T>::Handle(const MessageType& msg) {
  sim::data::JointCommands cmd;
  sim::converts::Converts(cmd, msg);

  if (IsEnable()) {
    sim::mj::SetWaistJointCommand(cmd);  // SIM API
  }

  JOINT_TOOLS_PRINT_N_INFO(1, "Subscribe Waist Joint Position: {}", sim::joint::tools::GetNamePosition(cmd));

  if (IsPrintable()) {
    AIMRT_INFO("Subscribe Waist Joint Position: {}", sim::joint::tools::GetNamePosition(cmd));
  }
}

}  // namespace aimrt::module::subscribe