#include "converts/data/lidar.hpp"

#include <cmath>
#include <limits>
#include <vector>

namespace ausim::converts {
namespace {

constexpr float kDegreesToRadians = static_cast<float>(M_PI) / 180.0f;
constexpr float kRangeEpsilon = 1e-4f;

bool IsValidHit(float range, float range_min_m, float range_max_m) {
  if (!std::isfinite(range) || range <= 0.0f) {
    return false;
  }
  if (std::isfinite(range_min_m) && range < range_min_m) {
    return false;
  }
  if (std::isfinite(range_max_m) && range >= range_max_m - kRangeEpsilon) {
    return false;
  }
  return true;
}

float BeamAngleRadians(float fov_deg, int count, int index) {
  if (count <= 1) {
    return 0.0f;
  }
  const float fov_rad = fov_deg * kDegreesToRadians;
  const float step = fov_rad / static_cast<float>(count - 1);
  return fov_rad * 0.5f - static_cast<float>(index) * step;
}

}  // namespace

std::vector<float> BuildLidarPointCloudXYZ(const ipc::LidarPacket& packet) {
  const int h = packet.h_ray_num;
  const int v = packet.v_ray_num;
  const int total = h * v;
  if (h <= 0 || v <= 0 || total > ipc::LidarPacket::kMaxRays) {
    return {};
  }

  std::vector<float> xyz;
  xyz.reserve(total * 3);

  for (int vi = 0; vi < v; ++vi) {
    const float elev = BeamAngleRadians(packet.fov_v_deg, v, vi);
    const float sin_elev = std::sin(elev);
    const float cos_elev = std::cos(elev);

    for (int hi = 0; hi < h; ++hi) {
      const float range = packet.data[vi * h + hi];
      if (!IsValidHit(range, packet.range_min_m, packet.range_max_m)) {
        continue;
      }

      const float azim = BeamAngleRadians(packet.fov_h_deg, h, hi);
      const float cos_azim = std::cos(azim);
      const float sin_azim = std::sin(azim);

      xyz.push_back(range * cos_elev * cos_azim);
      xyz.push_back(range * cos_elev * sin_azim);
      xyz.push_back(range * sin_elev);
    }
  }

  return xyz;
}

}  // namespace ausim::converts
