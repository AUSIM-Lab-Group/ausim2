
#include "./foot_sensor_publisher.h"
#include "aima/sim/converts/joint/joint_state.h"
#include "aima/sim/data/imu.hpp"
#include "imu_data_publisher.h"

#include "aima/sim/converts/converts.h"

namespace aimrt::module::publish {

template <class T>
ImuFootDataPublisher<T>::ImuFootDataPublisher() : BasePublisher() {}

template <class T>
bool ImuFootDataPublisher<T>::Init() {
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
void ImuFootDataPublisher<T>::Publish() {
  if (!publisher_) {
    return;
  }

  // todo delete
  return;

  // 从 sim 获取 imu 数据
  //   std_msgs::msg::Float64MultiArray imu_foot;
  std::vector<std::vector<double>> foot_sensor_data;

  // sim::mj::GetFootSensorData(foot_sensor_data);
  MessageType msg;
  sim::converts::Converts(msg, foot_sensor_data);

  if (IsEnable()) {
    aimrt::channel::Publish(publisher_, msg);
  }

  if (IsPrintable()) {
    AIMRT_INFO("Publish ImuData: {}", std_msgs::msg::to_yaml(msg));
  }
}

}  // namespace aimrt::module::publish