#pragma once

#include <memory>
#include <string>
#include "aima/sim/manager/aimrt/aimrt_helper.h"

#include "./service_cfg.h"

namespace aimrt::module::service {

class ServiceImpl {
  using BaseServicePtr = std::shared_ptr<aimrt::rpc::CoServiceBase>;

 protected:
  aimrt::CoreRef core_;
  ServiceConfig config_;
  std::atomic<bool> running_{false};

 private:
  BaseServicePtr service_;
  aimrt::executor::ExecutorRef executor_;

 public:
  ServiceImpl(BaseServicePtr service);
  virtual ~ServiceImpl() = default;

  void SetCore(aimrt::CoreRef core);
  void SetConfig(const ServiceConfig& config);
  std::string GetName() const;
  bool IsEnable() const;
  bool IsRunning() const;
  bool IsPrintable() const;

  bool Init();
  void Start();
  void Shutdown();
};

using ServiceImplPtr = std::shared_ptr<ServiceImpl>;

class ServiceFactory {
 public:
  using CreateServiceFunction = std::function<ServiceImplPtr()>;

  static ServiceFactory& Instance();
  void Register(const std::string& name, CreateServiceFunction func);
  ServiceImplPtr Create(const std::string& name);

 private:
  ServiceFactory() = default;
  ~ServiceFactory() = default;
};

#define REGISTER_SERVICE_IMPL(sub_name, sub_class)                                                                                      \
  static inline bool _g_##sub_name##_service_registered_ = []() {                                                                       \
    static_assert(std::is_base_of_v<aimrt::rpc::ServiceBase, sub_class>, "service_class must be derived from aimrt::rpc::ServiceBase"); \
    aimrt::module::service::ServiceFactory::Instance().Register(#sub_name, []() -> aimrt::module::service::ServiceImplPtr {             \
      return std::make_shared<aimrt::module::service::ServiceImpl>(std::make_shared<sub_class>());                                      \
    });                                                                                                                                 \
    return true;                                                                                                                        \
  }();

}  // namespace aimrt::module::service
