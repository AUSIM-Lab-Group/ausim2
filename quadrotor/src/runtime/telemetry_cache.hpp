#pragma once

#include <mutex>
#include <optional>

#include "runtime/runtime_types.hpp"

namespace quadrotor {

class TelemetryCache {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  void Write(const TelemetrySnapshot& snapshot);
  std::optional<TelemetrySnapshot> ReadLatest() const;

 private:
  mutable std::mutex mutex_;
  std::optional<TelemetrySnapshot> latest_snapshot_;
};

}  // namespace quadrotor
