#include "./manipulator_time_executor.h"
#include <thread>

#include "aimrt_module_cpp_interface/co/aimrt_context.h"
#include "aimrt_module_cpp_interface/co/schedule.h"
#include "aimrt_module_cpp_interface/co/sync_wait.h"

namespace aimrt::module::executor {

ManipulatorTimeExecutor::ManipulatorTimeExecutor() {}

void ManipulatorTimeExecutor::Init(aimrt::executor::ExecutorRef executor) {
  AIMRT_CHECK_ERROR_THROW(executor, "Get executor failed.");
  executor_ = executor;
}

void ManipulatorTimeExecutor::Start() {
  try {
    is_running_ = true;

    if (workers_.empty()) {
      AIMRT_WARN("ManipulatorTimeExecutor '{}' has no publisher, return.", executor_.Name());
      return;
    }

    // 启动

    for (auto worker : workers_) {
      auto scope = std::make_unique<aimrt::co::AsyncScope>();
      scope->spawn(LoopWorker(worker));
      scopes_.push_back(std::move(scope));
    }

  } catch (const std::exception& e) {
    AIMRT_ERROR("Start failed, {}", e.what());
    is_running_ = false;
  }

  AIMRT_INFO("ManipulatorTimeExecutor '{}' Started Succeeded.", executor_.Name());
}

void ManipulatorTimeExecutor::Shutdown() {
  is_running_ = false;

  for (auto& scope : scopes_) {
    aimrt::co::SyncWait(scope->complete());
  }

  AIMRT_INFO("ManipulatorTimeExecutor '{}' Shutdown.", executor_.Name());

  while (!workers_.empty()) {
    auto worker = workers_.back();
    workers_.pop_back();
    delete worker;
  }

  AIMRT_INFO("ManipulatorTimeExecutor '{}' Shutdown Succeeded.", executor_.Name());
}

void ManipulatorTimeExecutor::ResetWorkerTime(BaseWorkerPtr worker, const BaseWorker::ChronoTimePoint& now) {
  // 重置时间
  worker->UpdateWorkerTime(now);
}

aimrt::co::Task<void> ManipulatorTimeExecutor::LoopWorker(BaseWorkerPtr worker) {
  aimrt::co::AimRTScheduler scheduler(executor_);
  co_await aimrt::co::Schedule(scheduler);
  auto now = executor_.Now();
  worker->UpdateWorkerTime(now);
  while (is_running_) {
    co_await aimrt::co::ScheduleAt(scheduler, worker->GetWakeTime());
    now = executor_.Now();
    worker->UpdateWorkerTime(now);
    worker->Run();
  }
  co_return;
}

}  // namespace aimrt::module::executor
