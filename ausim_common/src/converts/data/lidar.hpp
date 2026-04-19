#pragma once

#include <vector>

#include "ipc/lidar_packet.hpp"

namespace ausim::converts {

// Returns a flat xyz buffer in lidar_link frame, 3 floats per point.
std::vector<float> BuildLidarPointCloudXYZ(const ipc::LidarPacket& packet);

}  // namespace ausim::converts
