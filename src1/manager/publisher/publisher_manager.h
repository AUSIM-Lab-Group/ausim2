#pragma once

#include <unordered_map>

#include "aima/sim/manager/executor/manipulator_time_executor.h"
#include "aima/sim/manager/executor/time_wheel_executor.h"

namespace aimrt::module::publish {

class PublisherManager {
  using TimeWheelPublisherExecutorPtr = std::shared_ptr<executor::TimeWheelExecutor>;
  using ManipulatorTimePublisherExecutorPtr = std::shared_ptr<executor::ManipulatorTimeExecutor>;

 public:
  PublisherManager(aimrt::CoreRef core);
  ~PublisherManager();

  void Init();

  void Start();

  void Shutdown();

 private:
  aimrt::CoreRef core_;
  std::unordered_map<std::string, TimeWheelPublisherExecutorPtr> time_wheel_publisher_executors_;
  std::unordered_map<std::string, ManipulatorTimePublisherExecutorPtr> manipulator_time_publisher_executors_;
};

}  // namespace aimrt::module::publish
