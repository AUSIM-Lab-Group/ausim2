
#include "./neck_joint_state_publisher.h"
#include "./joint_state_tools.inc"
#include "aima/sim/converts/joint/joint_state.h"
#include "aima/sim/module/mujoco/interface.h"

namespace aimrt::module::publish {

template <class T>
NeckJointStatePublisher<T>::NeckJointStatePublisher() : BasePublisher() {}

template <class T>
bool NeckJointStatePublisher<T>::Init() {
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
void NeckJointStatePublisher<T>::Publish() {
  if (!publisher_) {
    return;
  }

  // 从sim获取数据
  sim::data::JointStates state;
  sim::mj::GetHeadJointState(state);  // 模拟器API
  StateType msg;
  sim::converts::Converts(msg, state);

  if (IsEnable()) {
    aimrt::channel::Publish(publisher_, msg);
  }

  JOINT_TOOLS_PRINT_N_INFO(1, "Publish Neck Joint Position: {}", sim::joint::tools::GetNamePosition(state));

  if (IsPrintable()) {
    AIMRT_INFO("Publish Neck Joint Position: {}", sim::joint::tools::GetNamePosition(state));
  }
}

}  // namespace aimrt::module::publish