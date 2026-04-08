
#include "./core.h"

#include "aima/sim/module/mujoco/interface.h"
#include "co/on.h"
#include "log_util.h"
#include "yaml-cpp/yaml.h"

#include "aima/sim/module/publisher/publisher.h"
#include "aima/sim/module/subscriber/subscriber.h"

namespace sim::module {

void MujocoSimModule::OnConfigure(aimrte::ctx::ModuleCfg& cfg) {
  // cfg[aimrte::cfg::Module::Basic].Config(config_);
}

bool MujocoSimModule::OnInitialize() noexcept {
  // Save aimrt framework handle
  core_ = GetCoreRef();
  SetLogger(core_.GetLogger());

  try {
    aimrt::configurator::ConfiguratorRef configurator = core_.GetConfigurator();
    if (configurator) {
      std::string file_path = std::string(configurator.GetConfigFilePath());

      if (!file_path.empty()) {
        param::ReadParam(config_, file_path);
        AIMRT_INFO("MujocoSimModule Config:\n{}", param::ToString(config_));
      } else {
        AIMRT_ERROR_THROW("Configurator file path is empty.");
      }

    } else {
      AIMRT_ERROR_THROW("Get configurator failed.");
    }

    publisher_manager_ptr_ = std::make_shared<aimrt::module::publish::PublisherManager>(core_);
    AIMRT_CHECK_ERROR_THROW(publisher_manager_ptr_, "create publisher manager failed.");
    publisher_manager_ptr_->Init();

    subscriber_manager_ptr_ = std::make_shared<aimrt::module::subscribe::SubscriberManager>(core_);
    AIMRT_CHECK_ERROR_THROW(subscriber_manager_ptr_, "create subscriber manager failed.");
    subscriber_manager_ptr_->Init();

    // Get executor handle
    gui_executor_ = core_.GetExecutorManager().GetExecutor(config_.gui_executor);
    AIMRT_CHECK_ERROR_THROW(gui_executor_, "Get executor '{}' failed.", config_.gui_executor);

    sim_executor_ = core_.GetExecutorManager().GetExecutor(config_.sim_executor);
    AIMRT_CHECK_ERROR_THROW(sim_executor_, "Get executor '{}' failed.", config_.sim_executor);

    auto ok = sim_controller_.Init(config_.simulation_model, config_.robot_model, gui_executor_, sim_executor_);

    AIMRT_INFO("Init MujocoSimModule {} ...", ok ? "Succeeded" : "Failed");

  } catch (const std::exception& e) {
    AIMRT_HL_ERROR(core_.GetLogger(), "Init failed, {}", e.what());
    return false;
  }

  AIMRT_HL_INFO(core_.GetLogger(), "Init succeeded.");

  return true;
}

bool MujocoSimModule::OnStart() noexcept {
  try {
    is_running_ = true;
    sim_controller_.Start();

    while (!sim_controller_.IsRunning()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    subscriber_manager_ptr_->Start();
    publisher_manager_ptr_->Start();

  } catch (const std::exception& e) {
    AIMRT_HL_ERROR(core_.GetLogger(), "Start failed, {}", e.what());
    return false;
  }

  AIMRT_INFO("MujocoSimModule Started Succeeded ...");
  return true;
}

void MujocoSimModule::OnShutdown() noexcept {
  try {
    is_running_ = false;
    if (publisher_manager_ptr_) {
      publisher_manager_ptr_->Shutdown();
    }

    if (subscriber_manager_ptr_) {
      subscriber_manager_ptr_->Shutdown();
    }

    sim_controller_.Shutdown();
  } catch (const std::exception& e) {
    AIMRT_HL_ERROR(core_.GetLogger(), "Shutdown failed, {}", e.what());
    return;
  }

  AIMRT_INFO("MujocoSimModule Shutdown Succeeded ...");
}

}  // namespace sim::module