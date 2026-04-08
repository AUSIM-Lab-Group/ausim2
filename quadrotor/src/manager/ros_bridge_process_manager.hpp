#pragma once

#include <atomic>
#include <filesystem>
#include <string>
#include <thread>
#include <vector>

#include "config/quadrotor_config.hpp"

namespace quadrotor {

struct RosBridgeLaunchConfig {
  std::filesystem::path executable_path;
  std::vector<std::string> config_arguments;
};

class RosBridgeProcessManager {
 public:
  RosBridgeProcessManager(
      const QuadrotorConfig& config,
      RosBridgeLaunchConfig launch_config);
  ~RosBridgeProcessManager();

  void Start();
  void Stop();

 private:
  void TelemetryLoop();
  void CommandLoop();
  void EnsureChildStillRunning(const char* stage) const;

  QuadrotorConfig config_;
  RosBridgeLaunchConfig launch_config_;
  int telemetry_send_fd_ = -1;
  int command_recv_fd_ = -1;
  int child_pid_ = -1;
  std::atomic_bool running_ = false;
  std::thread telemetry_thread_;
  std::thread command_thread_;
};

}  // namespace quadrotor
