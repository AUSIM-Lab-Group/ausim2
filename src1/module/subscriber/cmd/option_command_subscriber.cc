
#include "./option_command_subscriber.h"
#include "aima/sim/common/db/data_board.h"
#include "aima/sim/data/op_cmd.hpp"

namespace aimrt::module::subscribe {

OptionCommandSubscriber::OptionCommandSubscriber() : BaseSubscriber() {}

bool OptionCommandSubscriber::Init() {
  if (config_.topic.empty()) {
    AIMRT_ERROR("Empty topic name.");
    return false;
  }

  Subscriber<MessageType>(std::bind(&OptionCommandSubscriber::Handle, this, std::placeholders::_1));
  return true;
}

void OptionCommandSubscriber::Handle(const MessageType& msg) {
  if (msg.data.empty()) {
    AIMRT_WARN("Received empty option command.");
    return;
  }
  static auto db = aimrte::db::DataBoard().Write<sim::data::OptionCommand>();
  auto cmd_db = db();
  if (msg.data == "enable_imu") {
    cmd_db.enable_pub_imu = true;

  } else if (msg.data == "disable_imu") {
    cmd_db.enable_pub_imu = false;
  }

  db = cmd_db;
}

}  // namespace aimrt::module::subscribe