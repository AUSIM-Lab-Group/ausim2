#pragma once

#include <time.h>
#include <chrono>
#include <cstdint>
#include <string>

namespace aimrt::module::executor {

class BaseWorker {
 public:
  using Clock = std::chrono::system_clock;
  using ChronoTimePoint = std::chrono::time_point<Clock>;

 private:
  std::string name;
  uint64_t period_ns;
  ChronoTimePoint wake_abs_time;

 public:
  BaseWorker(uint64_t period_ns, const std::string& name = "");
  virtual ~BaseWorker() = default;

  void ResetWakeTime();
  void UpdateWorkerTime(const ChronoTimePoint& now);

  uint64_t GetPeriod() const;
  const std::string& GetName() const;
  const ChronoTimePoint& GetWakeTime() const;
  [[nodiscard]] bool IsExpired(const ChronoTimePoint& now) const;
  bool operator<(BaseWorker& x) const;

  virtual void Run() = 0;
};

using BaseWorkerPtr = BaseWorker*;
struct BaseWorkerCompare {
  bool operator()(BaseWorkerPtr x, BaseWorkerPtr y) const;
};

}  // namespace aimrt::module::executor