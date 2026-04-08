#include <glfw_adapter.h>
#include <mujoco/mujoco.h>
#include <simulate.h>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <mutex>
#include <vector>
#include "aimrt_module_cpp_interface/co/sync_wait.h"

#include "./plugin_loader.h"
#include "./simulation_controller.h"
#include "co/aimrt_context.h"
#include "co/schedule.h"

namespace sim::mj {

SimulationController::SimulationController() {}
SimulationController::~SimulationController() {}

bool SimulationController::Init(const std::string& simulation_model, const std::string& model_info, aimrt::executor::ExecutorRef gui_executor,
                                aimrt::executor::ExecutorRef sim_executor) {
  model_info_ = model_info;
  simulation_model_ = simulation_model;
  gui_executor_ = gui_executor;
  sim_executor_ = sim_executor;
  return true;
}

void SimulationController::Start() {
  gui_scope_.spawn(GuiLoop());
  sim_scope_.spawn(SimLoop());
}

bool SimulationController::IsRunning() const { return is_running_; }

void SimulationController::Shutdown() {
  if (is_running_) {
    is_running_ = false;
    sim_->exitrequest.store(1);
  }
  aimrt::co::SyncWait(gui_scope_.complete());
  aimrt::co::SyncWait(sim_scope_.complete());
}

aimrt::co::Task<void> SimulationController::GuiLoop() {
  aimrt::co::AimRTScheduler gui_scheduler(gui_executor_);
  co_await aimrt::co::Schedule(gui_scheduler);

  // scan for libraries in the plugin directory to load additional plugins
  PluginLoader::ScanPlugins();

  mjvCamera cam;
  mjv_defaultCamera(&cam);

  mjvOption opt;
  mjv_defaultOption(&opt);

  mjvPerturb pert;
  mjv_defaultPerturb(&pert);

  sim_ = std::make_shared<mujoco::Simulate>(std::make_unique<mujoco::GlfwAdapter>(), &cam, &opt, &pert, /* is_passive = */ false);
  model_manager_ = std::make_shared<ModelManager>(sim_, model_info_);

  sim_->RenderLoop();

  if (is_running_) {
    std::raise(SIGTERM);
  }
  co_return;
}

aimrt::co::Task<void> SimulationController::SimLoop() {
  aimrt::co::AimRTScheduler sim_scheduler(sim_executor_);
  co_await aimrt::co::Schedule(sim_scheduler);

  while (model_manager_ == nullptr) {
    co_await aimrt::co::ScheduleAfter(sim_scheduler, std::chrono::milliseconds(10));
  }

  model_manager_->LoadModel(simulation_model_);
  auto now = sim_executor_.Now();
  wake_abs_time_ = now + std::chrono::nanoseconds(1000000);

  is_running_ = true;
  while (!sim_->exitrequest.load()) {
    if (sim_->droploadrequest.load()) {
      sim_->droploadrequest.store(false);
    }
    // 处理UI加载请求
    if (sim_->uiloadrequest.load()) {
      simulation_model_ = sim_->filename;
      model_manager_->LoadModel(simulation_model_);
      sim_->uiloadrequest.store(0);  // 多次点击无效
    }

    now = sim_executor_.Now();
    if (wake_abs_time_ < now) {
      wake_abs_time_ = now + std::chrono::nanoseconds(1000000);
    } else {
      co_await aimrt::co::ScheduleAt(sim_scheduler, wake_abs_time_);
      wake_abs_time_ += std::chrono::nanoseconds(1000000);
    }

    if (sim_->exitrequest.load()) {
      break;
    }

    auto model = model_manager_->GetModel();
    auto data = model_manager_->GetData();
    if (model == nullptr || data == nullptr) {
      continue;
    }

    model_manager_->UpdateModelCommand();

    {
      const std::unique_lock<std::recursive_mutex> lock(sim_->mtx);
      if (sim_->run) {
        mj_step(model, data);
        sim_->AddToHistory();
      } else {
        // paused
        mj_forward(model, data);
      }
    }

    model_manager_->UpdateModelState();
  }

  co_return;
}

}  // namespace sim::mj
