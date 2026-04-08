#include "./hand_joint_state_publisher.h"
#include "./joint_state_tools.inc"
#include "aima/sim/converts/joint/hand_state.h"
#include "aima/sim/module/mujoco/interface.h"

namespace aimrt::module::publish {

template <class T>
HandJointStatePublisher<T>::HandJointStatePublisher() : BasePublisher() {}

template <class T>
bool HandJointStatePublisher<T>::Init() {
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
void HandJointStatePublisher<T>::Publish() {
  if (!publisher_) {
    return;
  }

  // 从sim获取数据
  sim::data::JointStates state;
  // sim::api::GetHandJointState(state);  // SIM API
  sim::mj::GetHandJointState(state);  // 模拟器API
  StateType msg;
  sim::converts::Converts(msg, state);

  if (IsEnable()) {
    aimrt::channel::Publish(publisher_, msg);
  }

  JOINT_TOOLS_PRINT_N_INFO(1, "Publish Hand Joint Position: {}", sim::joint::tools::GetNamePosition(state));

  if (IsPrintable()) {
    AIMRT_INFO("Publish Hand Joint Position: {}", sim::joint::tools::GetNamePosition(state));
  }
}

}  // namespace aimrt::module::publish