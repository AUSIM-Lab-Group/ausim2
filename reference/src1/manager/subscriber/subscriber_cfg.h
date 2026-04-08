#pragma once

#include "aima/sim/common/parameter/param_reader.h"

namespace aimrt::module::subscribe {

struct SubscriberConfig {
  std::string name;         // 订阅器名
  std::string topic;        // 话题名
  bool enable = false;      // 使能
  std::string executor;     // 执行器
  bool wait_ready = false;  // 等待就绪
  int print_interval = 0;   // 打印间隔
};
PARAM_REFLECTION(SubscriberConfig, name, topic, enable, executor, wait_ready, print_interval);

struct SubscriberConfigList {
  std::list<SubscriberConfig> subscriber_cfg_list;
};
PARAM_REFLECTION(SubscriberConfigList, subscriber_cfg_list);

}  // namespace aimrt::module::subscribe