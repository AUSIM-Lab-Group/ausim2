
#include "./waist_joint_state_publisher.h"
#include "./joint_state_tools.inc"
#include "aima/sim/converts/joint/joint_state.h"
#include "aima/sim/module/mujoco/interface.h"

namespace aimrt::module::publish {

template <class T>
WaistJointStatePublisher<T>::WaistJointStatePublisher() : BasePublisher() {}

template <class T>
bool WaistJointStatePublisher<T>::Init() {
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
void WaistJointStatePublisher<T>::Publish() {
  if (!publisher_) {
    return;
  }

  // 从motion control API获取WaistJointState
  StateType msg;
  sim::data::JointStates state;
  sim::mj::GetWaistJointState(state);  // 模拟器API
  sim::converts::Converts(msg, state);

  if (IsEnable()) {
    aimrt::channel::Publish(publisher_, msg);
  }

  JOINT_TOOLS_PRINT_N_INFO(1, "Publish Waist Joint Position: {}", sim::joint::tools::GetNamePosition(state));

  if (IsPrintable()) {
    AIMRT_INFO("Publish Waist Joint Position: {}", sim::joint::tools::GetNamePosition(state));
  }
}

}  // namespace aimrt::module::publish