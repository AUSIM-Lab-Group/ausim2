#pragma once

#include <mujoco/mujoco.h>

#include "runtime/runtime_types.hpp"
#include "sim/mujoco_bindings.hpp"

namespace quadrotor {

class MujocoStateReader {
 public:
  RuntimeInput Read(const mjModel* model, const mjData* data, const MujocoBindings& bindings) const;
};

}  // namespace quadrotor
