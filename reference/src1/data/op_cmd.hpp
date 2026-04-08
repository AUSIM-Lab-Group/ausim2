#pragma once

namespace sim::data {

struct OptionCommand {
  bool enable_pub_imu = true;
  bool enable_pub_joint = true;
};

}  // namespace sim::data