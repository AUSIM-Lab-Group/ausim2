#include "./service_manager.h"
#include "./service_cfg.h"

namespace aimrt::module::service {

ServiceManager::ServiceManager(aimrt::CoreRef core) : core_(core) {}

ServiceManager::~ServiceManager() { Shutdown(); }

void ServiceManager::Init() {
  ServiceConfigList config;
  aimrt::configurator::ConfiguratorRef configurator = core_.GetConfigurator();
  if (configurator) {
    std::string file_path = std::string(configurator.GetConfigFilePath());
    if (!file_path.empty()) {
      param::ReadParam(config, file_path, "ServiceManager");
      AIMRT_INFO("ServiceManager Config:\n{}", param::ToString(config));
    }
  } else {
    AIMRT_ERROR_THROW("Get configurator failed.");
  }

  // 遍历配置列表，创建所有的服务
  for (const auto& cfg : config.service_cfg_list) {
    // 创建订阅器
    auto ptr = ServiceFactory::Instance().Create(cfg.name);
    if (ptr == nullptr) {
      AIMRT_ERROR("Create service: {} failed.", cfg.name);
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

    // 添加到列表
    services_.push_back(ptr);

    AIMRT_INFO("Add service '{}' to service manager.", cfg.name);
  }
}

void ServiceManager::Start() const {
  // 启动所有服务
  for (auto& it : services_) {
    it->Start();
  }
}

void ServiceManager::Shutdown() const {
  // 关闭所有订阅器
  for (auto& it : services_) {
    it->Shutdown();
  }
}

}  // namespace aimrt::module::service
