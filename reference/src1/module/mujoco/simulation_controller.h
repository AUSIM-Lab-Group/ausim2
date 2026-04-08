#pragma once

#include <mujoco/mujoco.h>
#include <simulate.h>
#include <atomic>
#include <memory>
#include <string>

#include "./model_manager.h"
#include "aimrt_module_cpp_interface/co/async_scope.h"
#include "aimrt_module_cpp_interface/co/task.h"
#include "aimrt_module_cpp_interface/executor/executor.h"

namespace sim::mj {

class SimulationController {
 public:
  explicit SimulationController();
  ~SimulationController();

  bool Init(const std::string& simulation_model, const std::string& model_info, aimrt::executor::ExecutorRef gui_executor,
            aimrt::executor::ExecutorRef sim_executor);

  void Start();

  void Shutdown();

  bool IsRunning() const;

 private:
  aimrt::co::Task<void> GuiLoop();
  aimrt::co::Task<void> SimLoop();

 private:
  std::string model_info_;
  std::string simulation_model_;
  std::atomic<bool> is_running_{false};
  aimrt::co::AsyncScope gui_scope_;
  aimrt::co::AsyncScope sim_scope_;
  std::shared_ptr<mujoco::Simulate> sim_;
  aimrt::executor::ExecutorRef gui_executor_;
  aimrt::executor::ExecutorRef sim_executor_;
  std::shared_ptr<sim::mj::ModelManager> model_manager_;
  std::chrono::time_point<std::chrono::system_clock> wake_abs_time_;
};

}  // namespace sim::mj