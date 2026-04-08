#pragma once

#include <memory>
#include <string>
#include "aima/sim/manager/aimrt/aimrt_helper.h"

#include "./client_cfg.h"

namespace aimrt::module::client {

class BaseClient {
 protected:
  aimrt::CoreRef core_;
  ClientConfig config_;
  std::atomic_bool running_{false};

 private:
  std::atomic<int> print_interval_{0};

 public:
  BaseClient(){};
  virtual ~BaseClient(){};

  void SetCore(aimrt::CoreRef core);
  void SetConfig(const ClientConfig& config);
  std::string GetName() const;
  bool IsEnable() const;
  bool IsPrintable();
  void PrintCount();

  virtual bool Init() = 0;
  virtual aimrt::co::Task<void> Handle(aimrt::executor::ExecutorRef executor) = 0;
};

using BaseClientPtr = std::shared_ptr<aimrt::module::client::BaseClient>;

class ClientFactory {
 public:
  using CreateClientFunction = std::function<BaseClientPtr()>;  // 创建一个BaseClient对象

  static ClientFactory& Instance();
  void Register(const std::string& name, CreateClientFunction func);
  BaseClientPtr Create(const std::string& name);

 private:
  ClientFactory() = default;
  ~ClientFactory() = default;
};

#define REGISTER_CLIENT_IMPL(cli_name, cli_class)                                                                                  \
  static inline bool _g_##cli_name##_client_registered_ = []() {                                                                   \
    static_assert(std::is_base_of_v<aimrt::module::client::BaseClient, cli_class>, "cli_class must be derived from BaseClient");   \
    aimrt::module::client::ClientFactory::Instance().Register(                                                                     \
        #cli_name, []() -> aimrt::module::client::BaseClientPtr { return std::make_shared<aimrt::module::client::cli_class>(); }); \
    return true;                                                                                                                   \
  }();

#define REGISTER_CLIENT_TEMPLATE_IMPL(cli_name, cli_class) \
  template class cli_class;                                \
  REGISTER_CLIENT_IMPL(cli_name, cli_class)

}  // namespace aimrt::module::client
