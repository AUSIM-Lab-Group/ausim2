#pragma once

#include <string>

#include "data/common.hpp"

namespace quadrotor::data {

struct OdomData {
  Header header;
  std::string child_frame_id;
  Pose pose;
  Twist twist;
};

}  // namespace quadrotor::data
