#include "./time_wheel_executor.h"

#include "aimrt_module_cpp_interface/co/aimrt_context.h"
#include "aimrt_module_cpp_interface/co/schedule.h"
#include "aimrt_module_cpp_interface/co/sync_wait.h"

namespace aimrt::module::executor {

TimeWheelExecutor::TimeWheelExecutor() {}

void TimeWheelExecutor::Init(aimrt::executor::ExecutorRef executor) {
  AIMRT_CHECK_ERROR_THROW(executor, "Get executor failed.");
  executor_ = executor;
}

void TimeWheelExecutor::Start() {
  try {
    is_running_ = true;

    if (queue_.empty()) {
      AIMRT_WARN("TimeWheelExecutor '{}' has no publisher, return.", executor_.Name());
      return;
    }

    // 启动
    scope_.spawn(Loop());
  } catch (const std::exception& e) {
    AIMRT_ERROR("Start failed, {}", e.what());
    is_running_ = false;
  }

  AIMRT_INFO("TimeWheelExecutor '{}' Started Succeeded.", executor_.Name());
}

void TimeWheelExecutor::Shutdown() {
  is_running_ = false;

  aimrt::co::SyncWait(scope_.complete());
  AIMRT_INFO("TimeWheelExecutor '{}' Shutdown.", executor_.Name());

  while (!queue_.empty()) {
    auto worker = queue_.top();
    queue_.pop();
    delete worker;
  }

  AIMRT_INFO("TimeWheelExecutor '{}' Shutdown Succeeded.", executor_.Name());
}

void TimeWheelExecutor::ResetWorkerTime(const BaseWorker::ChronoTimePoint& now) {
  // 重置时间
  std::vector<BaseWorkerPtr> workers;
  while (!queue_.empty()) {
    auto worker = queue_.top();
    worker->ResetWakeTime();
    auto now = BaseWorker::Clock::now();
    worker->UpdateWorkerTime(now);
    workers.push_back(worker);
    queue_.pop();
  }
  for (auto worker : workers) {
    queue_.push(worker);
  }
}

aimrt::co::Task<void> TimeWheelExecutor::Loop() {
  if (queue_.empty()) {
    co_return;
  }

  aimrt::co::AimRTScheduler scheduler(executor_);
  co_await aimrt::co::Schedule(scheduler);

  auto now = BaseWorker::Clock::now();
  ResetWorkerTime(now);
  while (is_running_) {
    // 获取优先级队列中最快要到期的worker
    auto worker = queue_.top();
    if (worker->IsExpired(now)) {
      // 更新
      worker->UpdateWorkerTime(now);
      // Work
      worker->Run();
      // 重新入队
      queue_.pop();
      queue_.push(worker);
    } else {
      // 等待下一个周期
      std::this_thread::sleep_until(worker->GetWakeTime());
      now = BaseWorker::Clock::now();
    }
  }

  co_return;
}

}  // namespace aimrt::module::executor
