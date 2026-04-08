#pragma once

#include <string>

namespace sim::mj {

class PluginLoader {
 public:
  static void ScanPlugins();
  static std::string GetExecutableDir();
};

}  // namespace sim::mj
