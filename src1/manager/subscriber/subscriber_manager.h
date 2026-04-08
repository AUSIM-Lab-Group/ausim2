#pragma once

#include <vector>

#include "./subscriber_base.h"

namespace aimrt::module::subscribe {

class SubscriberManager {
 public:
  SubscriberManager(aimrt::CoreRef core);
  ~SubscriberManager();

  void Init();

  void Start() const;

  void Shutdown();

  bool AllReady() const;

 private:
  aimrt::CoreRef core_;
  std::vector<BaseSubscriberPtr> subscribers_;
};

}  // namespace aimrt::module::subscribe
