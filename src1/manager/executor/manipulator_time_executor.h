#pragma once

#include <atomic>
#include <memory>
#include <vector>
#include "./worker.h"

#include "aima/sim/manager/aimrt/aimrt_helper.h"
#include "aimrt_module_cpp_interface/co/async_scope.h"
#include "aimrt_module_cpp_interface/co/task.h"

namespace aimrt::module::executor {

class ManipulatorTimeExecutor {
 public:
  ManipulatorTimeExecutor();
  ~ManipulatorTimeExecutor() = default;

  void Init(aimrt::executor::ExecutorRef executor);
  void Start();
  void Shutdown();

  template <typename T, typename... ConstructorArgs>
  void AddWorker(ConstructorArgs&&... args) {
    static_assert(std::is_base_of<BaseWorker, T>::value, "T must be derived from BaseWorker");
    auto worker = new T(std::forward<ConstructorArgs>(args)...);
    workers_.push_back(worker);
    AIMRT_INFO("Add worker '{}' to executor '{}', period: {} us", worker->GetName(), executor_.Name(), worker->GetPeriod() / 1000);
    AIMRT_INFO("Executor '{}' has {} workers.", executor_.Name(), workers_.size());
  }

 private:
  void ResetWorkerTime(BaseWorkerPtr worker, const BaseWorker::ChronoTimePoint& now);
  co::Task<void> LoopWorker(BaseWorkerPtr worker);

 private:
  std::list<std::unique_ptr<aimrt::co::AsyncScope>> scopes_;
  std::atomic<bool> is_running_{false};
  std::vector<BaseWorkerPtr> workers_;
  aimrt::executor::ExecutorRef executor_;
};

}  // namespace aimrt::module::executor
