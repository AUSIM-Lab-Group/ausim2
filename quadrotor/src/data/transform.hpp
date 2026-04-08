#pragma once

#include <string>

#include "data/common.hpp"

namespace quadrotor::data {

struct TransformData {
  Header header;
  std::string child_frame_id;
  Vector3 translation;
  Quaternion rotation;
};

}  // namespace quadrotor::data
