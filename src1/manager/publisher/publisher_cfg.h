#pragma once

#include <string>

namespace aimrt::module::publish {

struct PublisherConfig {
  std::string name;        // 发布器名
  std::string topic;       // 话题名
  bool enable = false;     // 使能
  double frequency;        // 发布频率
  int print_interval = 0;  // 打印间隔
};

}  // namespace aimrt::module::publish