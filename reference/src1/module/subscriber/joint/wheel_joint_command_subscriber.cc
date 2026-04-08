
#include "./wheel_joint_command_subscriber.h"
#include "./joint_command_tools.inc"
#include "aima/sim/converts/converts.h"
#include "aima/sim/module/mujoco/interface.h"

namespace aimrt::module::subscribe {

template <class T>
WheelJointCommandSubscriber<T>::WheelJointCommandSubscriber() : BaseSubscriber() {}

template <class T>
bool WheelJointCommandSubscriber<T>::Init() {
  if (config_.topic.empty()) {
    AIMRT_ERROR("Empty topic name.");
    return false;
  }

  Subscriber<MessageType>(std::bind(&WheelJointCommandSubscriber::Handle, this, std::placeholders::_1));
  return true;
}

template <class T>
void WheelJointCommandSubscriber<T>::Handle(const MessageType& msg) {
  sim::data::JointCommands cmd;
  sim::converts::Converts(cmd, msg);

  if (IsEnable()) {
    // sim::api::SetWheelJointCommand(cmd);  // SIM API
    sim::mj::SetWheelJointCommand(cmd);  // SIM API
  }

  if (IsPrintable()) {
    std::cout << "cmd size is: " << cmd.joints.size() << std::endl;

    for (int i = 0; i < msg.joints.size(); i++) {
      std::cout << "name: " << msg.joints[i].name << ", position: " << msg.joints[i].position << ", velocity: " << msg.joints[i].velocity
                << std::endl;
    }
    std::cout << std::endl;

    AIMRT_INFO("Subscribe Wheel Joint Position: {}", sim::joint::tools::GetNamePosition(cmd));
    AIMRT_INFO("Subscribe Wheel Joint Velocity: {}", sim::joint::tools::GetNameVelocity(cmd));
  }
}

}  // namespace aimrt::module::subscribe