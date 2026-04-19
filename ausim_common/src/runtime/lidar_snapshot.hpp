#pragma once

#include <vector>

namespace ausim {

// In-process container for a single lidar scan written by the simulator.
struct LidarSnapshot {
  double sim_time = 0.0;
  int h_ray_num = 0;
  int v_ray_num = 0;
  float fov_h_deg = 360.0f;
  float fov_v_deg = 30.0f;
  // Distance values in metres, size == h_ray_num * v_ray_num.
  // Index: h * v_ray_num + v.  Infinity means no hit.
  std::vector<float> ranges;
};

}  // namespace ausim
