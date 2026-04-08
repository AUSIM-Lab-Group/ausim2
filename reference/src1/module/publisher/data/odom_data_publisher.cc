#include "./odom_data_publisher.h"
#include "aima/sim/common/db/data_board.h"
#include "aima/sim/data/odom.hpp"
#include "aima/sim/data/op_cmd.hpp"
#include "aima/sim/module/mujoco/interface.h"  // 用于调用GetOdomData接口

namespace aimrt::module::publish {

template <class T>
OdomDataPublisher<T>::OdomDataPublisher() : BasePublisher() {}

template <class T>
bool OdomDataPublisher<T>::Init() {
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
void OdomDataPublisher<T>::Publish() {
  if (!publisher_) {
    return;
  }

  sim::data::OdomData odom;
  sim::mj::GetOdomData(odom);  // 调用Mujoco接口获取数据

  MessageType msg;
  sim::converts::Converts(msg, odom);

  if (IsEnable()) {
    static auto db = aimrte::db::DataBoard().Read<sim::data::OptionCommand>();
    if (db().enable_pub_imu) {  // 可新增enable_pub_odom，此处先复用IMU开关
      aimrt::channel::Publish(publisher_, msg);
    }
  }

  if (IsPrintable()) {
    AIMRT_INFO("Publish ImuData: {}", nav_msgs::msg::to_yaml(msg));
  }
}

REGISTER_PUBLISHER_TEMPLATE_INSTANTIATION(OdomDataPublisher<nav_msgs::msg::Odometry>);

}  // namespace aimrt::module::publish