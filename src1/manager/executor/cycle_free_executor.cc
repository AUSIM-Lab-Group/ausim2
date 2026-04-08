#include "./cycle_free_executor.h"
#include "aimrt_module_cpp_interface/co/sync_wait.h"
#include "co/schedule.h"

namespace aimrt::module::executor {

CycleFreeExecutor::CycleFreeExecutor() {}

void CycleFreeExecutor::Init(aimrt::executor::ExecutorRef executor, int sleep_ms) {
  AIMRT_CHECK_ERROR_THROW(executor, "Get executor failed.");
  executor_ = executor;
  sleep_ms_ = sleep_ms;
}

void CycleFreeExecutor::Start() {
  try {
    is_running_ = true;

    if (workers_.empty()) {
      AIMRT_WARN("CycleFreeExecutor '{}' has no publisher, return.", executor_.Name());
      return;
    }

    // 启动
    scope_.spawn(Loop());
  } catch (const std::exception& e) {
    AIMRT_ERROR("Start failed, {}", e.what());
    is_running_ = false;
  }

  AIMRT_INFO("CycleFreeExecutor '{}' Started Succeeded.", executor_.Name());
}

void CycleFreeExecutor::Shutdown() {
  is_running_ = false;

  aimrt::co::SyncWait(scope_.complete());
  AIMRT_INFO("CycleFreeExecutor '{}' Shutdown.", executor_.Name());

  for (auto worker : workers_) {
    delete worker;
  }
  workers_.clear();

  AIMRT_INFO("CycleFreeExecutor '{}' Shutdown Succeeded.", executor_.Name());
}

aimrt::co::Task<void> CycleFreeExecutor::Loop() {
  if (workers_.empty()) {
    co_return;
  }

  while (is_running_) {
    aimrt::co::AimRTScheduler scheduler(executor_);
    co_await aimrt::co::ScheduleAfter(scheduler, std::chrono::milliseconds(sleep_ms_));

    for (auto worker : workers_) {
      co_await worker->Run(executor_);
    }
  }

  co_return;
}

}  // namespace aimrt::module::executor