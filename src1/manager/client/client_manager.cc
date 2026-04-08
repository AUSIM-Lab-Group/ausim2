#include "./client_manager.h"
#include "./client_cfg.h"
#include "aima/sim/common/parameter/param_reader.h"
#include "aima/sim/manager/executor/cycle_free_executor.h"
#include "client_base.h"

namespace aimrt::module::client {

namespace details {

struct CompleteClientConfig : public ClientConfig {
  std::string executor;  // 执行器名
};
PARAM_REFLECTION(CompleteClientConfig, name, enable, executor, print_interval, sleep_ms);

struct ClientConfigList {
  std::list<details::CompleteClientConfig> client_cfg_list;
};
PARAM_REFLECTION(ClientConfigList, client_cfg_list);

class ClientWorker : public executor::CycleFreeBaseWorker {
 private:
  BaseClientPtr client;

 public:
  virtual aimrt::co::Task<void> Run(aimrt::executor::ExecutorRef executor) override {
    client->PrintCount();
    co_await client->Handle(executor);
    co_return;
  }

  ClientWorker(BaseClientPtr client) : executor::CycleFreeBaseWorker(client->GetName()), client(client) {}
};

}  // namespace details

ClientManager::ClientManager(aimrt::CoreRef core) : core_(core) {}

ClientManager::~ClientManager() { Shutdown(); }

void ClientManager::Init() {
  details::ClientConfigList config;
  aimrt::configurator::ConfiguratorRef configurator = core_.GetConfigurator();
  if (configurator) {
    std::string file_path = std::string(configurator.GetConfigFilePath());
    if (!file_path.empty()) {
      param::ReadParam(config, file_path, "ClientManager");
      AIMRT_INFO("ClientManager Config:\n{}", param::ToString(config));
    }
  } else {
    AIMRT_ERROR_THROW("Get configurator failed.");
  }

  // 遍历配置列表，获取所有发布器配置的 executor
  for (const auto& cfg : config.client_cfg_list) {
    if (!client_executors_.contains(cfg.executor)) {
      // 如果该 executor 不存在，则创建一个新的发布器执行器
      auto executor_ptr = std::make_shared<executor::CycleFreeExecutor>();
      // 初始化发布器执行器
      executor_ptr->Init(core_.GetExecutorManager().GetExecutor(cfg.executor), cfg.sleep_ms);
      client_executors_[cfg.executor] = executor_ptr;
    }
  }

  // 遍历配置列表，创建所有的服务
  for (const auto& cfg : config.client_cfg_list) {
    // 创建订阅器
    auto ptr = ClientFactory::Instance().Create(cfg.name);
    if (ptr == nullptr) {
      AIMRT_ERROR("Create client: {} failed.", cfg.name);
      continue;
    }

    // 设置核心
    ptr->SetCore(core_);
    // 设置配置
    ptr->SetConfig(cfg);

    // 初始化
    if (ptr->Init() == false) {
      continue;
    }

    // 添加到client执行器
    client_executors_[cfg.executor]->AddWorker<details::ClientWorker>(ptr);
    AIMRT_INFO("Add client '{}' to client manager.", cfg.name);
  }
}

void ClientManager::Start() const {
  // 启动所有服务
  for (auto& it : client_executors_) {
    it.second->Start();
  }
}

void ClientManager::Shutdown() const {
  // 关闭所有订阅器
  for (auto& it : client_executors_) {
    it.second->Shutdown();
  }
}

}  // namespace aimrt::module::client
