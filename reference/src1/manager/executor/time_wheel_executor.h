#pragma once

#include <atomic>
#include "./worker.h"

#include "aima/sim/manager/aimrt/aimrt_helper.h"
#include "aimrt_module_cpp_interface/co/async_scope.h"
#include "aimrt_module_cpp_interface/co/task.h"

namespace aimrt::module::executor {

class TimeWheelExecutor {
 public:
  TimeWheelExecutor();
  ~TimeWheelExecutor() = default;

  void Init(aimrt::executor::ExecutorRef executor);
  void Start();
  void Shutdown();

  template <typename T, typename... ConstructorArgs>
  void AddWorker(ConstructorArgs&&... args) {
    static_assert(std::is_base_of<BaseWorker, T>::value, "T must be derived from BaseWorker");
    auto worker = new T(std::forward<ConstructorArgs>(args)...);
    queue_.push(worker);
    AIMRT_INFO("Add worker '{}' to executor '{}', period: {} us", worker->GetName(), executor_.Name(), worker->GetPeriod() / 1000);
    AIMRT_INFO("Executor '{}' has {} workers.", executor_.Name(), queue_.size());
  }

 private:
  void ResetWorkerTime(const BaseWorker::ChronoTimePoint& now);
  co::Task<void> Loop();

 private:
  aimrt::co::AsyncScope scope_;
  std::atomic<bool> is_running_{false};
  aimrt::executor::ExecutorRef executor_;
  std::priority_queue<BaseWorkerPtr, std::vector<BaseWorkerPtr>, BaseWorkerCompare> queue_;
};

}  // namespace aimrt::module::executor
