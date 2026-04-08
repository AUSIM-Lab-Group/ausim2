#include "./plugin_loader.h"
#include <mujoco/mujoco.h>
#include <filesystem>
#include <iostream>

namespace sim::mj {

std::string PluginLoader::GetExecutableDir() { return std::filesystem::current_path().string(); }

void PluginLoader::ScanPlugins() {
  // check and print plugins that are linked directly into the executable
  int nplugin = mjp_pluginCount();
  if (nplugin) {
    std::printf("Built-in plugins:\n");
    for (int i = 0; i < nplugin; ++i) {
      std::printf("    %s\n", mjp_getPluginAtSlot(i)->name);
    }
  }

  // try to open the ${EXECDIR}/MUJOCO_PLUGIN_DIR directory
  // ${EXECDIR} is the directory containing the simulate binary itself
  // MUJOCO_PLUGIN_DIR is the MUJOCO_PLUGIN_DIR preprocessor macro
  std::string executable_dir = GetExecutableDir();
  if (executable_dir.empty()) {
    return;
  }
  auto plugin_dir = executable_dir + "/" + "mujoco_plugin";
  mj_loadAllPluginLibraries(
      plugin_dir.c_str(), +[](const char *filename, int first, int count) {
        std::printf("Plugins from '%s':\n", filename);
        for (int i = first; i < first + count; ++i) {
          std::printf("    %s\n", mjp_getPluginAtSlot(i)->name);
        }
      });
}

}  // namespace sim::mj
