#pragma once

#include <cstdint>
#include <type_traits>

namespace ausim::ipc {

// Fixed-size wire format for lidar scan data sent over the lidar socketpair.
// kMaxRays must be >= h_ray_num * v_ray_num for any supported configuration.
struct LidarPacket {
  double sim_time = 0.0;
  int32_t h_ray_num = 0;
  int32_t v_ray_num = 0;
  float fov_h_deg = 360.0f;
  float fov_v_deg = 30.0f;
  // Pad to 360 * 32 = 11520 entries (≈ 46 KB). Actual valid entries: h_ray_num * v_ray_num.
  static constexpr int kMaxRays = 360 * 32;
  float data[kMaxRays] = {};
};

static_assert(std::is_trivially_copyable_v<LidarPacket>);

}  // namespace ausim::ipc
