#include "./publisher_manager.h"
#include "./publisher_base.h"
#include "./publisher_cfg.h"
#include "aima/sim/common/parameter/param_reader.h"

namespace aimrt::module::publish {

namespace details {
struct CompletePublisherConfig : public PublisherConfig {
  std::string executor;  // 执行器名
};
PARAM_REFLECTION(CompletePublisherConfig, name, topic, enable, frequency, executor, print_interval);

struct PublisherConfigList {
  std::list<details::CompletePublisherConfig> publisher_cfg_list;
};
PARAM_REFLECTION(PublisherConfigList, publisher_cfg_list);

}  // namespace details

class PublisherWorker : public executor::BaseWorker {
 private:
  BasePublisherPtr publisher;

 public:
  void Run() override {
    publisher->PrintCount();
    publisher->Publish();
  }

  PublisherWorker(BasePublisherPtr publisher)
      : executor::BaseWorker((uint64_t)(1000000000.0 / publisher->GetFrequency()), publisher->GetName()), publisher(publisher) {}
};

PublisherManager::PublisherManager(aimrt::CoreRef core) : core_(core) {}

PublisherManager::~PublisherManager() { Shutdown(); }

void PublisherManager::Init() {
  details::PublisherConfigList config;
  aimrt::configurator::ConfiguratorRef configurator = core_.GetConfigurator();
  if (configurator) {
    const auto file_path = std::string(configurator.GetConfigFilePath());
    if (!file_path.empty()) {
      param::ReadParam(config, file_path, "PublisherManager");
      AIMRT_INFO("PublisherManager Config:\n{}", param::ToString(config));
    }
  } else {
    AIMRT_ERROR_THROW("Get configurator failed.");
  }

  // 遍历配置列表，获取所有发布器配置的 executor
  for (const auto& cfg : config.publisher_cfg_list) {
    if (!time_wheel_publisher_executors_.contains(cfg.executor) && !manipulator_time_publisher_executors_.contains(cfg.executor)) {
      auto executor = core_.GetExecutorManager().GetExecutor(cfg.executor);
      if (!executor) {
        continue;
      }

      if (executor.SupportTimerSchedule()) {
        // 如果该 executor 不存在，则创建一个新的发布器执行器
        auto executor_ptr = std::make_shared<executor::ManipulatorTimeExecutor>();
        // 初始化发布器执行器
        executor_ptr->Init(executor);
        manipulator_time_publisher_executors_[cfg.executor] = executor_ptr;
      } else {
        // 如果该 executor 不存在，则创建一个新的发布器执行器
        auto executor_ptr = std::make_shared<executor::TimeWheelExecutor>();
        // 初始化发布器执行器
        executor_ptr->Init(executor);
        time_wheel_publisher_executors_[cfg.executor] = executor_ptr;
      }
    }
  }

  // 遍历配置列表，添加发布器
  for (const auto& cfg : config.publisher_cfg_list) {
    // 创建发布器
    BasePublisherPtr ptr = PublisherFactory::Instance().Create(cfg.name);
    if (ptr == nullptr) {
      AIMRT_ERROR("Create publisher: {} failed.", cfg.name);
      continue;
    }
    // 设置核心
    ptr->SetCore(core_);
    // 设置配置
    ptr->SetConfig(cfg);

    // 初始化发布器
    if (ptr->Init() == false) {
      continue;
    }

    // 添加到发布器执行器
    if (manipulator_time_publisher_executors_.contains(cfg.executor)) {
      manipulator_time_publisher_executors_[cfg.executor]->AddWorker<PublisherWorker>(ptr);
    } else if (time_wheel_publisher_executors_.contains(cfg.executor)) {
      time_wheel_publisher_executors_[cfg.executor]->AddWorker<PublisherWorker>(ptr);
    }

    AIMRT_INFO("Add publisher {} to executor: {}", cfg.name, cfg.executor);
  }
}

void PublisherManager::Start() {
  // 启动所有发布器执行器
  for (auto& it : manipulator_time_publisher_executors_) {
    it.second->Start();
  }

  for (auto& it : time_wheel_publisher_executors_) {
    it.second->Start();
  }
}

void PublisherManager::Shutdown() {
  // 关闭所有发布器执行器
  for (auto& it : manipulator_time_publisher_executors_) {
    it.second->Shutdown();
  }

  for (auto& it : time_wheel_publisher_executors_) {
    it.second->Shutdown();
  }

  manipulator_time_publisher_executors_.clear();
  time_wheel_publisher_executors_.clear();
}

}  // namespace aimrt::module::publish
