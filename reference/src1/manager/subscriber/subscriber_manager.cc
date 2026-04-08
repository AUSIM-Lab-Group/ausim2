#include "./subscriber_manager.h"

namespace aimrt::module::subscribe {

SubscriberManager::SubscriberManager(aimrt::CoreRef core) : core_(core) {}

SubscriberManager::~SubscriberManager() { Shutdown(); }

void SubscriberManager::Init() {
  SubscriberConfigList config;
  aimrt::configurator::ConfiguratorRef configurator = core_.GetConfigurator();
  if (configurator) {
    std::string file_path = std::string(configurator.GetConfigFilePath());
    if (!file_path.empty()) {
      param::ReadParam(config, file_path, "SubscriberManager");
      AIMRT_INFO("SubscriberManager Config:\n{}", param::ToString(config));
    }
  } else {
    AIMRT_ERROR_THROW("Get configurator failed.");
  }

  // 遍历配置列表，创建所有的订阅器
  for (const auto& cfg : config.subscriber_cfg_list) {
    // 创建订阅器
    BaseSubscriberPtr ptr = SubscriberFactory::Instance().Create(cfg.name);
    if (ptr == nullptr) {
      AIMRT_ERROR("Create subscriber: {} failed.", cfg.name);
      continue;
    }
    // 设置核心
    ptr->SetCore(core_);
    // 设置配置
    ptr->SetConfig(cfg);

    // 初始化订阅器
    if (ptr->Init() == false) {
      continue;
    }

    // 添加订阅器到订阅器列表
    subscribers_.push_back(ptr);

    AIMRT_INFO("Create subscriber: {} succeeded, topic: {}", cfg.name, cfg.topic);
  }
}

void SubscriberManager::Start() const {
  // 启动所有订阅器
  for (auto& it : subscribers_) {
    it->Start();
  }
}

void SubscriberManager::Shutdown() {
  // 关闭所有订阅器
  for (auto& it : subscribers_) {
    it->Shutdown();
  }
  subscribers_.clear();
}

bool SubscriberManager::AllReady() const {
  // 检查所有订阅器是否就绪
  for (const auto& it : subscribers_) {
    if (!it->IsReady()) {
      AIMRT_INFO("Waiting for {} topic '{}' ready ...", it->GetName(), it->GetTopic());
      return false;
    }
  }
  return true;
}

}  // namespace aimrt::module::subscribe
