#include "runtime/telemetry_cache.hpp"

namespace quadrotor {

void TelemetryCache::Write(const TelemetrySnapshot& snapshot) {
  const std::lock_guard<std::mutex> lock(mutex_);
  latest_snapshot_ = snapshot;
}

std::optional<TelemetrySnapshot> TelemetryCache::ReadLatest() const {
  const std::lock_guard<std::mutex> lock(mutex_);
  return latest_snapshot_;
}

}  // namespace quadrotor
