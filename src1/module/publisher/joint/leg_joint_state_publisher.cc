
#include "./leg_joint_state_publisher.h"
#include "./joint_state_tools.inc"
#include "aima/sim/converts/joint/joint_state.h"
#include "aima/sim/module/mujoco/interface.h"

namespace aimrt::module::publish {

template <class T>
LegJointStatePublisher<T>::LegJointStatePublisher() : BasePublisher() {}

template <class T>
bool LegJointStatePublisher<T>::Init() {
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
void LegJointStatePublisher<T>::Publish() {
  if (!publisher_) {
    return;
  }

  // 从 sim 获取LegJointState
  sim::data::JointStates state;
  // sim::api::GetLegJointState(state);  // SIM API
  sim::mj::GetLegJointState(state);  // 模拟器API
  StateType msg;
  sim::converts::Converts(msg, state);

  if (IsEnable()) {
    aimrt::channel::Publish(publisher_, msg);
  }

  JOINT_TOOLS_PRINT_N_INFO(1, "Publish Leg Joint Position: {}", sim::joint::tools::GetNamePosition(state));

  if (IsPrintable()) {
    AIMRT_INFO("Publish Leg Joint Effort: {}", sim::joint::tools::GetNamePosition(state));
  }
}

}  // namespace aimrt::module::publish