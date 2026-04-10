#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <string>

#include <mujoco/mujoco.h>

namespace fs = std::filesystem;

namespace {

fs::path GetPluginDir(int argc, char** argv) {
  if (argc > 1 && argv[1] != nullptr && argv[1][0] != '\0') {
    return fs::path(argv[1]);
  }
  return fs::path(MUJOCO_RAY_CASTER_PLUGIN_DIR);
}

fs::path GetModelPath(int argc, char** argv) {
  if (argc > 2 && argv[2] != nullptr && argv[2][0] != '\0') {
    return fs::path(argv[2]);
  }
  return fs::path(MUJOCO_RAY_CASTER_SMOKETEST_MODEL);
}

}  // namespace

int main(int argc, char** argv) {
  const fs::path plugin_dir = GetPluginDir(argc, argv);
  const fs::path model_path = GetModelPath(argc, argv);

  if (!fs::exists(plugin_dir)) {
    std::cerr << "Plugin directory does not exist: " << plugin_dir << std::endl;
    return 1;
  }
  if (!fs::exists(model_path)) {
    std::cerr << "Smoke test model does not exist: " << model_path << std::endl;
    return 1;
  }

  const int plugin_count_before = mjp_pluginCount();
  mj_loadAllPluginLibraries(plugin_dir.string().c_str(), nullptr);
  const int plugin_count_after = mjp_pluginCount();
  if (plugin_count_after <= plugin_count_before) {
    std::cerr << "No MuJoCo plugins were loaded from: " << plugin_dir << std::endl;
    return 1;
  }

  char load_error[1024] = {0};
  mjModel* model = mj_loadXML(model_path.string().c_str(), nullptr, load_error, sizeof(load_error));
  if (model == nullptr) {
    std::cerr << "Failed to load smoke test model: " << load_error << std::endl;
    return 1;
  }

  mjData* data = mj_makeData(model);
  if (data == nullptr) {
    std::cerr << "Failed to allocate mjData for smoke test model." << std::endl;
    mj_deleteModel(model);
    return 1;
  }

  mj_forward(model, data);

  const int sensor_id = mj_name2id(model, mjOBJ_SENSOR, "raycasterlidar");
  if (sensor_id < 0) {
    std::cerr << "Smoke test model did not expose the expected sensor 'raycasterlidar'." << std::endl;
    mj_deleteData(data);
    mj_deleteModel(model);
    return 1;
  }

  const int sensor_adr = model->sensor_adr[sensor_id];
  std::cout << "Loaded " << (plugin_count_after - plugin_count_before)
            << " plugin(s) from " << plugin_dir << '\n'
            << "Smoke test model loaded successfully: " << model_path << '\n'
            << "raycasterlidar sensor address: " << sensor_adr << '\n'
            << "first sensor sample: " << data->sensordata[sensor_adr] << std::endl;

  mj_deleteData(data);
  mj_deleteModel(model);
  return 0;
}
