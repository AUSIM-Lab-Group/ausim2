#pragma once

#include <memory>
#include <string>
#include "aima/sim/manager/aimrt/aimrt_helper.h"

#include "./subscriber_cfg.h"

namespace aimrt::module::subscribe {

class BaseSubscriber {
 protected:
  aimrt::CoreRef core_;
  SubscriberConfig config_;
  std::atomic<bool> running_{false};

 private:
  std::atomic<bool> ready_{false};
  std::atomic<int> print_interval_{0};
  aimrt::executor::ExecutorRef executor_;
  aimrt::channel::SubscriberRef subscriber_;

 public:
  BaseSubscriber();
  virtual ~BaseSubscriber() = default;

  void SetCore(aimrt::CoreRef core);

  void SetConfig(const SubscriberConfig& config);
  std::string GetName() const;
  std::string GetTopic() const;
  bool IsEnable() const;
  bool IsRunning() const;
  bool IsReady() const;
  void Ready();
  void Start();
  void Shutdown();
  bool IsPrintable();
  void PrintCount();

  template <typename T>
  void Subscriber(std::function<void(const T&)>&& callback) {
    subscriber_ = core_.GetChannelHandle().GetSubscriber(config_.topic);
    AIMRT_CHECK_ERROR_THROW(subscriber_, "Get subscriber for topic '{}' failed.", config_.topic);

    executor_ = core_.GetExecutorManager().GetExecutor(config_.executor);
    AIMRT_CHECK_ERROR_THROW(executor_, "Get executor for name '{}' failed.", config_.executor);

    auto ret = aimrt::channel::SubscribeCo<T>(subscriber_, [this, callback(std::move(callback))](const T& msg) -> aimrt::co::Task<void> {
      if (!IsRunning()) co_return;
      aimrt::co::AimRTScheduler scheduler(executor_);
      co_await aimrt::co::Schedule(scheduler);
      try {
        PrintCount();
        callback(msg);
        Ready();
      } catch (const std::exception& e) {
        AIMRT_ERROR("SubscriberWrapper callback error: {}", e.what());
      }
      co_return;
    });
    AIMRT_CHECK_ERROR_THROW(ret, "Subscribe failed.");
  }

  virtual bool Init() = 0;
};

using BaseSubscriberPtr = std::shared_ptr<aimrt::module::subscribe::BaseSubscriber>;

class SubscriberFactory {
 public:
  using CreateSubscriberFunction = std::function<BaseSubscriberPtr()>;

  static SubscriberFactory& Instance();
  void Register(const std::string& name, CreateSubscriberFunction func);
  BaseSubscriberPtr Create(const std::string& name);

 private:
  SubscriberFactory() = default;
  ~SubscriberFactory() = default;
};

#define REGISTER_SUBSCRIBER_IMPL(sub_name, sub_class)                                                                                              \
  static inline bool _g_##sub_name##_subscriber_registered_ = []() {                                                                               \
    static_assert(std::is_base_of_v<aimrt::module::subscribe::BaseSubscriber, sub_class>, "subscriber_class must be derived from BaseSubscriber"); \
    aimrt::module::subscribe::SubscriberFactory::Instance().Register(#sub_name,                                                                    \
                                                                     []() -> BaseSubscriberPtr { return std::make_shared<sub_class>(); });         \
    return true;                                                                                                                                   \
  }();

#define REGISTER_SUBSCRIBER_TEMPLATE_IMPL(sub_name, sub_class) \
  template class sub_class;                                    \
  REGISTER_SUBSCRIBER_IMPL(sub_name, sub_class)

}  // namespace aimrt::module::subscribe
