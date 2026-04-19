#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

#include <mujoco/mujoco.h>

#include "config/scout_config.hpp"
#include "sim/scout_sim.hpp"

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
    if (fs::exists(current / "ground_vehicle" / "cfg" / "sim_config.yaml")) {
      return current;
    }
    if (!current.has_parent_path()) {
      break;
    }
    current = current.parent_path();
  }
  return {};
}

int FindDynamicObstacleGeom(const mjModel* model) {
  if (model == nullptr) {
    return -1;
  }
  for (int geom_id = 0; geom_id < model->ngeom; ++geom_id) {
    const char* name = mj_id2name(model, mjOBJ_GEOM, geom_id);
    if (name != nullptr && std::string(name).rfind("dynamic_obs_", 0) == 0) {
      return geom_id;
    }
  }
  return -1;
}

}  // namespace

int main() {
  const fs::path repo_root = ResolveRepoRoot();
  Expect(!repo_root.empty(), "failed to locate repo root for dynamic_obstacle_test");

  const fs::path obstacle_config_path = fs::temp_directory_path() / "scout_dynamic_obstacle_test.yaml";
  std::ofstream output(obstacle_config_path);
  output << R"yaml(
random_seed: 1
dynamic: true
collision_enabled: false
debug: false
mode: "2d"
range:
  x_min: 0.5
  x_max: 20.5
  y_min: -5.0
  y_max: 5.0
  z_min: 0.0
  z_max: 1.5
obstacle_count: 20
min_speed: 0.3
max_speed: 0.3
update_threads: 1
parallel_threshold: 16
)yaml";
  output.close();

  ground_vehicle::ScoutConfig config =
      ground_vehicle::LoadScoutConfigFromYaml((repo_root / "ground_vehicle" / "cfg" / "sim_config.yaml").string(), "");
  config.common.viewer.enabled = false;
  config.common.viewer.fallback_to_headless = true;
  config.common.model.scene_xml = repo_root / "assets" / "scout_v2" / "scene.dynamic_obstacles.xml";
  config.common.dynamic_obstacle.enabled = true;
  config.common.dynamic_obstacle.config_path = obstacle_config_path.string();

  ground_vehicle::ScoutSim sim(config);
  sim.LoadModel();

  const mjModel* model = sim.model();
  Expect(model != nullptr, "expected Scout model to load");
  const int geom_id = FindDynamicObstacleGeom(model);
  Expect(geom_id >= 0, "expected scene.dynamic_obstacles.xml to contain dynamic_obs_* geoms");

  const int adr = geom_id * 3;
  const double start_x = model->geom_pos[adr + 0];
  const double start_y = model->geom_pos[adr + 1];
  const double start_z = model->geom_pos[adr + 2];

  for (int i = 0; i < 20; ++i) {
    sim.Step();
  }

  const double end_x = model->geom_pos[adr + 0];
  const double end_y = model->geom_pos[adr + 1];
  const double end_z = model->geom_pos[adr + 2];

  Expect(start_x != end_x || start_y != end_y || start_z != end_z,
         "expected at least one dynamic obstacle geom position to change after stepping Scout simulation");

  fs::remove(obstacle_config_path);
  return 0;
}
