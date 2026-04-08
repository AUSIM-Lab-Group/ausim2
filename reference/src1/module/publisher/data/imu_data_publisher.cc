
#include "./imu_data_publisher.h"
// #include "aima/sim/converts/joint/joint_state.h"
#include "aima/sim/common/db/data_board.h"
#include "aima/sim/data/imu.hpp"
#include "aima/sim/data/op_cmd.hpp"
#include "aima/sim/module/mujoco/interface.h"

namespace aimrt::module::publish {

template <class T>
ImuDataPublisher<T>::ImuDataPublisher() : BasePublisher() {}

template <class T>
bool ImuDataPublisher<T>::Init() {
  if (config_.frequency <= 0) {
    AIMRT_ERROR("Invalid frequency: {}", config_.frequency);
    return false;
  }

  if (config_.topic.empty()) {
    AIMRT_ERROR("Empty topic name.");
    return false;
  }

  publisher_ = aimrt::helper::GetPublisher<MessageType>(core_, config_.topic);
  if (!publisher_) {
    AIMRT_ERROR("Failed to get publisher: {}", config_.topic.c_str());
    return false;
  }
  return true;
}

template <class T>
void ImuDataPublisher<T>::Publish() {
  if (!publisher_) {
    return;
  }
  std::string publisher_name = config_.topic;
  // std::cout << "Publisher name: " << publisher_name << std::endl;
  const std::string imu_frame_type = (config_.topic == "/aima/hal/imu/torso/state") ? "imu" : "imu_1";
  // 从 sim 获取 imu 数据
  sim::data::ImuData imu;
  sim::mj::GetImuData(imu, imu_frame_type);
  MessageType msg;
  sim::converts::Converts(msg, imu);

  if (IsEnable()) {
    static auto db = aimrte::db::DataBoard().Read<sim::data::OptionCommand>();
    if (db().enable_pub_imu) {
      aimrt::channel::Publish(publisher_, msg);
    }
  }

  if (IsPrintable()) {
    AIMRT_INFO("Publish ImuData: {}", sensor_msgs::msg::to_yaml(msg));
  }
}

REGISTER_PUBLISHER_TEMPLATE_INSTANTIATION(ImuDataPublisher<sensor_msgs::msg::Imu>);

}  // namespace aimrt::module::publish