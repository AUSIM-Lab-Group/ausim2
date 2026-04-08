// sim_mujoco_copy/data/odom.hpp
#pragma once

#include "aima/sim/data/imu.hpp"  // 复用Vec3和Quaternion

namespace sim::data {
struct OdomData {
  Vec3 position;           // 位置(x,y,z)
  Quaternion orientation;  // 姿态四元数(w,x,y,z)
  Vec3 linear_velocity;    // 线速度
  Vec3 angular_velocity;   // 角速度
};
}  // namespace sim::data