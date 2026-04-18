#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <string>

#include "config/quadrotor_config.hpp"
#include "runtime/data_board_interface.hpp"
#include "runtime/runtime_types.hpp"
#include "sim/quadrotor_sim.hpp"

namespace fs = std::filesystem;

namespace {

void Expect(bool condition, const std::string& message) {
  if (!condition) {
    std::cerr << message << '\n';
    std::exit(1);
  }
}

fs::path ResolveRepoRoot() {
  fs::path current = fs::current_path();
  for (int i = 0; i < 4; ++i) {
    if (fs::exists(current / "quadrotor" / "cfg" / "sim_config.yaml")) {
      return current;
    }
    if (!current.has_parent_path()) {
      break;
    }
    current = current.parent_path();
  }
  return {};
}

}  // namespace

int main() {
  const fs::path repo_root = ResolveRepoRoot();
  Expect(!repo_root.empty(), "failed to locate repo root for reset_simulation_test");

  quadrotor::ClearVelocityCommand();
  quadrotor::ClearDiscreteCommand();

  const fs::path sim_config_path = repo_root / "quadrotor" / "cfg" / "sim_config.yaml";
  quadrotor::QuadrotorConfig config = quadrotor::LoadConfigFromYaml(sim_config_path.string(), "");
  config.viewer.enabled = false;
  config.viewer.fallback_to_headless = true;

  quadrotor::QuadrotorSim sim(config);
  sim.LoadModel();

  sim.Step();
  sim.Step();
  Expect(sim.data() != nullptr, "sim data should exist before reset");
  Expect(sim.data()->time > 0.0, "sim time should advance before reset");

  quadrotor::DiscreteCommand reset_command;
  reset_command.event_name = "reset";
  reset_command.kind = quadrotor::DiscreteCommandKind::kResetSimulation;
  reset_command.sequence = 1;
  reset_command.received_time = std::chrono::steady_clock::now();
  quadrotor::WriteDiscreteCommand(reset_command);

  sim.Step();

  Expect(sim.data() != nullptr, "sim data should still exist after reset step");
  Expect(std::isfinite(sim.data()->time), "sim time should stay finite after reset");
  Expect(sim.data()->time <= config.simulation.dt + 1e-9, "reset should rewind sim time close to zero");

  sim.Step();
  Expect(std::isfinite(sim.data()->time), "sim should continue stepping after reset");

  quadrotor::ClearDiscreteCommand();
  quadrotor::ClearVelocityCommand();
  return 0;
}
