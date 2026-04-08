#include "./worker.h"
#include <cstdint>

#include "aima/sim/manager/aimrt/aimrt_helper.h"

namespace aimrt::module::executor {

BaseWorker::BaseWorker(uint64_t p, const std::string& n) : name(n), period_ns(p) { ResetWakeTime(); }

void BaseWorker::ResetWakeTime() { wake_abs_time = BaseWorker::Clock::now(); }

void BaseWorker::UpdateWorkerTime(const ChronoTimePoint& now) {
  if (period_ns <= 0) {
    return;
  }

  // 更新唤醒时间
  wake_abs_time += std::chrono::nanoseconds(period_ns);

  // 更新唤醒时间后，发现还小于当前时间，则重置时间
  if (IsExpired(now)) {
    wake_abs_time = now + std::chrono::nanoseconds(period_ns);
  }
}

[[nodiscard]] bool BaseWorker::IsExpired(const ChronoTimePoint& now) const { return (wake_abs_time < now); }

bool BaseWorker::operator<(BaseWorker& x) const {
  // return ((wake_abs_time.tv_sec > x.wake_abs_time.tv_sec) ||
  //         (wake_abs_time.tv_sec == x.wake_abs_time.tv_sec && wake_abs_time.tv_nsec > x.wake_abs_time.tv_nsec));
  // > ? <
  return wake_abs_time > x.wake_abs_time;
}

uint64_t BaseWorker::GetPeriod() const { return period_ns; }
const std::string& BaseWorker::GetName() const { return name; }
const BaseWorker::ChronoTimePoint& BaseWorker::GetWakeTime() const { return wake_abs_time; }

bool BaseWorkerCompare::operator()(BaseWorkerPtr x, BaseWorkerPtr y) const { return *x < *y; }

}  // namespace aimrt::module::executor