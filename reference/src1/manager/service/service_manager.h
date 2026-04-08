#pragma once

#include <unordered_map>

#include "./service_base.h"

namespace aimrt::module::service {

class ServiceManager {
 public:
  ServiceManager(aimrt::CoreRef core);
  ~ServiceManager();

  void Init();

  void Start() const;

  void Shutdown() const;

 private:
  aimrt::CoreRef core_;
  std::list<ServiceImplPtr> services_;
};

}  // namespace aimrt::module::service
