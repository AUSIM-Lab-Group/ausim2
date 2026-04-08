#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>

#include "aima/sim/manager/aimrt/aimrt_helper.h"
#include "aimrt_module_cpp_interface/co/async_scope.h"
#include "aimrt_module_cpp_interface/co/task.h"
#include "executor/executor.h"

namespace aimrt::module::executor {

class CycleFreeBaseWorker {
 private:
  std::string name;

 public:
  CycleFreeBaseWorker(const std::string& name) : name(name) {}
  virtual ~CycleFreeBaseWorker() = default;

  const std::string& GetName() const { return name; }
  virtual aimrt::co::Task<void> Run(aimrt::executor::ExecutorRef executor) = 0;
};

class CycleFreeExecutor {
 public:
  CycleFreeExecutor();
  ~CycleFreeExecutor() = default;

  void Init(aimrt::executor::ExecutorRef executor, int sleep_ms = 10);
  void Start();
  void Shutdown();

  template <typename T, typename... ConstructorArgs>
  void AddWorker(ConstructorArgs&&... args) {
    static_assert(std::is_base_of<CycleFreeBaseWorker, T>::value, "T must be derived from CycleFreeBaseWorker");
    auto worker = new T(std::forward<ConstructorArgs>(args)...);
    workers_.push_back(worker);
    AIMRT_INFO("Add worker '{}' to executor '{}'", worker->GetName(), executor_.Name());
    AIMRT_INFO("Executor '{}' has {} workers.", executor_.Name(), workers_.size());
  }

 private:
  co::Task<void> Loop();

 private:
  int sleep_ms_;
  aimrt::co::AsyncScope scope_;
  std::atomic<bool> is_running_{false};
  aimrt::executor::ExecutorRef executor_;
  std::vector<CycleFreeBaseWorker*> workers_;
};

}  // namespace aimrt::module::executor
