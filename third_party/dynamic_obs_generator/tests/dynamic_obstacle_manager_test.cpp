#include "dynamic_obstacle_manager.hpp"

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>

#include <mujoco/mujoco.h>

namespace {

namespace fs = std::filesystem;
using dynamic_obstacle::DynamicObstacleManager;
using dynamic_obstacle::GenerationMode;
using dynamic_obstacle::ObstacleConfig;

class MujocoScene {
 public:
  explicit MujocoScene(const std::string& xml) {
    const fs::path path = fs::temp_directory_path() / "dynamic_obstacle_manager_test_scene.xml";
    {
      std::ofstream out(path);
      out << xml;
    }

    char error[1024] = "";
    model_ = mj_loadXML(path.string().c_str(), nullptr, error, sizeof(error));
    fs::remove(path);
    if (model_ == nullptr) {
      throw std::runtime_error(std::string("mj_loadXML failed: ") + error);
    }

    data_ = mj_makeData(model_);
    if (data_ == nullptr) {
      mj_deleteModel(model_);
      throw std::runtime_error("mj_makeData failed");
    }

    mj_resetData(model_, data_);
    mj_forward(model_, data_);
  }

  ~MujocoScene() {
    if (data_ != nullptr) {
      mj_deleteData(data_);
    }
    if (model_ != nullptr) {
      mj_deleteModel(model_);
    }
  }

  mjModel* model() const { return model_; }
  mjData* data() const { return data_; }

 private:
  mjModel* model_ = nullptr;
  mjData* data_ = nullptr;
};

ObstacleConfig MakeConfig() {
  ObstacleConfig config;
  config.random_seed = 1;
  config.dynamic = true;
  config.mode = GenerationMode::k2D;
  config.radius = 0.25;
  config.box_size = 0.5;
  config.range_x_min = -5.0;
  config.range_x_max = 5.0;
  config.range_y_min = -5.0;
  config.range_y_max = 5.0;
  config.range_z_min = 0.0;
  config.range_z_max = 2.0;
  config.obstacle_count = 4;
  config.min_speed = 1.0;
  config.max_speed = 1.0;
  config.update_threads = 1;
  config.parallel_threshold = 16;
  return config;
}

bool NearlyEqual(double lhs, double rhs, double tolerance = 1e-9) {
  return std::abs(lhs - rhs) <= tolerance;
}

void Require(bool condition, const std::string& message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

void TestMocapObstaclesUseMocapPoseUpdates() {
  const std::string xml = R"(
<mujoco>
  <worldbody>
    <body name="dynamic_obs_0" mocap="true" pos="1 2 0.5">
      <geom name="dynamic_obs_0_geom" type="box" size="0.2 0.2 0.2" contype="1" conaffinity="1"/>
    </body>
    <geom name="dynamic_obs_1" type="box" pos="2 0 0.5" size="0.2 0.2 0.2" contype="0" conaffinity="0"/>
  </worldbody>
</mujoco>
)";

  MujocoScene scene(xml);
  DynamicObstacleManager manager;
  ObstacleConfig config = MakeConfig();

  Require(manager.Initialize(config, scene.model(), scene.data()), "Initialize should succeed");
  Require(manager.GetObstacleCount() == 2, "Expected two dynamic obstacles in the scene");
  Require(!manager.RequiresPhysicsRateUpdates(),
          "Collidable mocap obstacles should not force physics-rate updates");

  const int body_id = mj_name2id(scene.model(), mjOBJ_BODY, "dynamic_obs_0");
  Require(body_id >= 0, "mocap body should exist");
  const int mocap_id = scene.model()->body_mocapid[body_id];
  Require(mocap_id >= 0, "mocap body should have a mocap id");
  const int mocap_addr = mocap_id * 3;
  const double initial_x = scene.data()->mocap_pos[mocap_addr + 0];
  const double initial_y = scene.data()->mocap_pos[mocap_addr + 1];

  const int geom_id = mj_name2id(scene.model(), mjOBJ_GEOM, "dynamic_obs_1");
  Require(geom_id >= 0, "direct geom obstacle should exist");
  const int geom_addr = geom_id * 3;
  const double initial_geom_x = scene.model()->geom_pos[geom_addr + 0];
  const double initial_geom_y = scene.model()->geom_pos[geom_addr + 1];

  Require(manager.ApplyTrajectory(1.0), "ApplyTrajectory should move dynamic obstacles");

  const double moved_mocap_x = scene.data()->mocap_pos[mocap_addr + 0];
  const double moved_mocap_y = scene.data()->mocap_pos[mocap_addr + 1];
  Require(!(NearlyEqual(initial_x, moved_mocap_x) && NearlyEqual(initial_y, moved_mocap_y)),
          "mocap obstacle should update data->mocap_pos");

  const double moved_geom_x = scene.model()->geom_pos[geom_addr + 0];
  const double moved_geom_y = scene.model()->geom_pos[geom_addr + 1];
  Require(!(NearlyEqual(initial_geom_x, moved_geom_x) && NearlyEqual(initial_geom_y, moved_geom_y)),
          "non-mocap obstacle should keep the direct geom_pos path");
}

void TestCollidableNonMocapObstacleWarns() {
  const std::string xml = R"(
<mujoco>
  <worldbody>
    <geom name="dynamic_obs_0" type="box" pos="1 0 0.5" size="0.2 0.2 0.2" contype="1" conaffinity="1"/>
  </worldbody>
</mujoco>
)";

  MujocoScene scene(xml);
  DynamicObstacleManager manager;
  ObstacleConfig config = MakeConfig();

  std::ostringstream captured_stderr;
  std::streambuf* original_stderr = std::cerr.rdbuf(captured_stderr.rdbuf());
  const bool initialized = manager.Initialize(config, scene.model(), scene.data());
  std::cerr.rdbuf(original_stderr);

  Require(initialized, "Initialize should succeed for collidable non-mocap obstacle");
  Require(manager.RequiresPhysicsRateUpdates(),
          "Collidable non-mocap obstacle should still force physics-rate updates");
  Require(captured_stderr.str().find("collidable dynamic_obs must use a mocap body") != std::string::npos,
          "Expected a warning for collidable non-mocap dynamic obstacles");
}

}  // namespace

int main() {
  try {
    TestMocapObstaclesUseMocapPoseUpdates();
    TestCollidableNonMocapObstacleWarns();
  } catch (const std::exception& ex) {
    std::cerr << ex.what() << std::endl;
    return 1;
  }

  return 0;
}
