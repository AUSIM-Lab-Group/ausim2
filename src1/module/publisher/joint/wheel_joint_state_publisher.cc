
#include "./wheel_joint_state_publisher.h"
#include "./joint_state_tools.inc"
#include "aima/sim/converts/joint/joint_state.h"
#include "aima/sim/module/mujoco/interface.h"

namespace aimrt::module::publish {

template <class T>
WheelJointStatePublisher<T>::WheelJointStatePublisher() : BasePublisher() {}

template <class T>
bool WheelJointStatePublisher<T>::Init() {
  if (config_.frequency <= 0) {
    AIMRT_ERROR("Invalid frequency: {}", config_.frequency);
    return false;
  }

  if (config_.topic.empty()) {
    AIMRT_ERROR("Empty topic name.");
    return false;
  }

  publisher_ = aimrt::helper::GetPublisher<StateType>(core_, config_.topic);
  if (!publisher_) {
    AIMRT_ERROR("Failed to get publisher: {}", config_.topic.c_str());
    return false;
  }
  return true;
}

template <class T>
void WheelJointStatePublisher<T>::Publish() {
  if (!publisher_) {
    return;
  }

  sim::data::JointStates state;
  // sim::api::GetWheelJointState(state);  // SIM API
  sim::mj::GetWheelJointState(state);  // SIM API
  StateType msg;
  sim::converts::Converts(msg, state);

  if (IsEnable()) {
    aimrt::channel::Publish(publisher_, msg);
  }

  if (IsPrintable()) {
    std::vector<std::string> wheel_names;
    AIMRT_INFO("Publish Wheel Joint Position: {}", sim::joint::tools::GetNamePosition(state));
    AIMRT_INFO("Publish Wheel Joint Velocity: {}", sim::joint::tools::GetNameVelocity(state));
  }
}

}  // namespace aimrt::module::publish