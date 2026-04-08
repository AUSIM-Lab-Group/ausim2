#pragma once

#include <atomic>
#include <chrono>
#include <filesystem>
#include <memory>
#include <string>
#include <thread>

#include <Eigen/Core>
#include <mujoco/mujoco.h>

#include "config/quadrotor_config.hpp"
#include "ros/ros2_bridge.hpp"
#include "runtime/command_mailbox.hpp"
#include "runtime/telemetry_cache.hpp"
#include "runtime/vehicle_runtime.hpp"
#include "sim/mujoco_actuator_writer.hpp"
#include "sim/mujoco_bindings.hpp"
#include "sim/mujoco_state_reader.hpp"

namespace mujoco {
class Simulate;
}

namespace quadrotor {

class QuadrotorSim {
 public:
  explicit QuadrotorSim(QuadrotorConfig config);
  ~QuadrotorSim();

  void LoadModel();
  void Step();
  void Run();
  bool viewer_enabled() const { return viewer_ != nullptr; }

  const mjModel* model() const { return model_; }
  const mjData* data() const { return data_; }

 private:
  static void ControlCallback(const mjModel* model, mjData* data);
  void ApplyControl(const mjModel* model, mjData* data);

  void ResetSimulation();
  void LogStateIfNeeded(const TelemetrySnapshot& snapshot) const;

  void RunHeadless();
  void RunWithViewer();
  void InitializeViewer();
  void CleanupViewer();
  void PhysicsThreadMain();
  void PhysicsLoop(mujoco::Simulate& sim);
  bool LoadModelIntoViewer(
      mujoco::Simulate& sim,
      const std::filesystem::path& model_path,
      bool replace_existing);
  void InstallModelPointers(
      mjModel* new_model,
      mjData* new_data,
      const std::filesystem::path& model_path,
      bool replace_existing);
  void ConfigureDefaultCamera();
  std::string ValidateModel(const mjModel* candidate) const;
  int ComputeControlDecimation() const;
  bool ShouldContinueHeadless() const;
  void SleepToMatchRealtime(
      const std::chrono::high_resolution_clock::time_point& step_start,
      double simulated_seconds) const;
  static void HandleSigint(int signal);

  QuadrotorConfig config_;
  std::shared_ptr<CommandMailbox> command_mailbox_;
  std::shared_ptr<TelemetryCache> telemetry_cache_;
  VehicleRuntime runtime_;
  MujocoBindings bindings_;
  MujocoStateReader state_reader_;
  MujocoActuatorWriter actuator_writer_;
  std::unique_ptr<Ros2Bridge> ros_bridge_;
  mjModel* model_ = nullptr;
  mjData* data_ = nullptr;
  std::unique_ptr<mujoco::Simulate> viewer_;
  std::thread physics_thread_;
  mjvCamera camera_;
  mjvOption visualization_options_;
  mjvPerturb perturbation_;
  mutable double next_log_time_ = 0.0;
  int control_decimation_ = 1;
  int control_step_count_ = 0;
  std::atomic_bool stop_requested_ = false;

  static QuadrotorSim* active_instance_;
};

}  // namespace quadrotor
