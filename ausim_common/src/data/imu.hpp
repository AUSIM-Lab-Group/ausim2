#pragma once

#include "data/common.hpp"

namespace quadrotor::data {

struct ImuData {
  Header header;
  Quaternion orientation;
  Vector3 angular_velocity;
  Vector3 linear_acceleration;
  bool has_linear_acceleration = false;
};

}  // namespace quadrotor::data
