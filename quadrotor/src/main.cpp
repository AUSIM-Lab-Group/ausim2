#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "sim/quadrotor_sim.hpp"

namespace fs = std::filesystem;

namespace {

struct CliOptions {
  fs::path merged_config_path;
  fs::path sim_config_path;
  fs::path robot_config_path;
  bool force_viewer = false;
  bool force_headless = false;
};

fs::path ResolveExistingPath(const std::vector<fs::path>& candidates) {
  for (const fs::path& candidate : candidates) {
    if (!candidate.empty() && fs::exists(candidate)) {
      return candidate.lexically_normal();
    }
  }
  return {};
}

fs::path ResolveDefaultConfigPath(const char* filename) {
  return ResolveExistingPath({
      fs::current_path() / "quadrotor" / "cfg" / filename,
      fs::current_path() / "cfg" / filename,
      fs::current_path() / ".." / "quadrotor" / "cfg" / filename,
      fs::path(QUADROTOR_SOURCE_DIR) / "cfg" / filename,
  });
}

void PrintUsage(const char* program_name) {
  std::cout << "Usage: " << program_name
            << " [--sim-config <path>] [--robot-config <path>] [--config <legacy.yaml>] "
               "[--viewer|--headless]\n";
}

}  // namespace

int main(int argc, char** argv) {
  try {
    CliOptions cli;

    for (int i = 1; i < argc; ++i) {
      const std::string arg = argv[i];
      if (arg == "--help" || arg == "-h") {
        PrintUsage(argv[0]);
        return 0;
      } else if (arg == "--config") {
        if (i + 1 >= argc) {
          throw std::runtime_error("--config requires a file path.");
        }
        cli.merged_config_path = argv[++i];
      } else if (arg == "--sim-config") {
        if (i + 1 >= argc) {
          throw std::runtime_error("--sim-config requires a file path.");
        }
        cli.sim_config_path = argv[++i];
      } else if (arg == "--robot-config") {
        if (i + 1 >= argc) {
          throw std::runtime_error("--robot-config requires a file path.");
        }
        cli.robot_config_path = argv[++i];
      } else if (arg == "--viewer") {
        cli.force_viewer = true;
      } else if (arg == "--headless") {
        cli.force_headless = true;
      } else {
        if (!cli.merged_config_path.empty()) {
          throw std::runtime_error("Only one positional legacy config path is supported.");
        }
        cli.merged_config_path = arg;
      }
    }

    if (!cli.merged_config_path.empty() &&
        (!cli.sim_config_path.empty() || !cli.robot_config_path.empty())) {
      throw std::runtime_error("Use either --config or the split --sim-config/--robot-config inputs.");
    }

    quadrotor::QuadrotorConfig config;
    if (!cli.merged_config_path.empty()) {
      config = quadrotor::LoadConfigFromYaml(cli.merged_config_path.string());
    } else {
      if (cli.sim_config_path.empty()) {
        cli.sim_config_path = ResolveDefaultConfigPath("sim_config.yaml");
      }
      if (cli.robot_config_path.empty()) {
        cli.robot_config_path = ResolveDefaultConfigPath("robot_config.yaml");
      }
      if (cli.sim_config_path.empty() || cli.robot_config_path.empty()) {
        throw std::runtime_error(
            "Unable to locate default config files. Expected quadrotor/cfg/sim_config.yaml "
            "and quadrotor/cfg/robot_config.yaml.");
      }
      config = quadrotor::LoadConfigFromYaml(
          cli.sim_config_path.string(),
          cli.robot_config_path.string());
    }

    if (cli.force_viewer) {
      config.viewer.enabled = true;
    }
    if (cli.force_headless) {
      config.viewer.enabled = false;
    }

    quadrotor::QuadrotorSim sim(std::move(config));
    sim.Run();
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "quadrotor error: " << error.what() << '\n';
    return 1;
  }
}
