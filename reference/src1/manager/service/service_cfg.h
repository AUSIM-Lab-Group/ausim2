#pragma once

#include "aima/sim/common/parameter/param_reader.h"

namespace aimrt::module::service {

struct ServiceConfig {
  std::string name;          // 服务名
  bool enable = false;       // 使能
  bool debug_print = false;  // 调试打印
  std::string executor;      // 执行器
};
PARAM_REFLECTION(ServiceConfig, name, enable, debug_print, executor);

struct ServiceConfigList {
  std::list<ServiceConfig> service_cfg_list;
};
PARAM_REFLECTION(ServiceConfigList, service_cfg_list);

}  // namespace aimrt::module::service