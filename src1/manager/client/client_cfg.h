#pragma once

#include <string>

namespace aimrt::module::client {

struct ClientConfig {
  std::string name;        // 服务名
  bool enable = false;     // 使能
  int print_interval = 0;  // 打印间隔
  int sleep_ms = 0;        // 休眠时间，单位 ms
};

}  // namespace aimrt::module::client