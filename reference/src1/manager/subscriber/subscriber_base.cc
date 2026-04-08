#include <unordered_map>

#include "./subscriber_base.h"

namespace aimrt::module::subscribe {

BaseSubscriber::BaseSubscriber() = default;

void BaseSubscriber::SetCore(aimrt::CoreRef core) { core_ = core; }
void BaseSubscriber::SetConfig(const SubscriberConfig& config) { config_ = config; }

std::string BaseSubscriber::GetName() const { return config_.name; }
std::string BaseSubscriber::GetTopic() const { return config_.topic; }
bool BaseSubscriber::IsEnable() const { return config_.enable; }
bool BaseSubscriber::IsRunning() const { return running_; }
bool BaseSubscriber::IsReady() const {
  if (!config_.wait_ready || !IsEnable()) {
    return true;
  }
  return ready_;
}

void BaseSubscriber::Start() { running_ = true; }
void BaseSubscriber::Shutdown() { running_ = false; }
void BaseSubscriber::Ready() { ready_ = true; }

bool BaseSubscriber::IsPrintable() {
  if (config_.print_interval > 0) {
    if (print_interval_ >= config_.print_interval) {
      print_interval_ = 0;
      return true;
    }
  }
  return false;
}

void BaseSubscriber::PrintCount() {
  if (config_.print_interval) {
    ++print_interval_;
  }
}

using SubscriberFunctionMap = std::unordered_map<std::string, SubscriberFactory::CreateSubscriberFunction>;
static std::shared_ptr<SubscriberFunctionMap> _g_subscriber_factory_map_{nullptr};

SubscriberFactory& SubscriberFactory::Instance() {
  static SubscriberFactory instance;
  if (!_g_subscriber_factory_map_) {
    _g_subscriber_factory_map_ = std::make_shared<SubscriberFunctionMap>();
  }
  return instance;
}

void SubscriberFactory::Register(const std::string& name, CreateSubscriberFunction func) {
  if (_g_subscriber_factory_map_->contains(name)) {
    return;
  }

  _g_subscriber_factory_map_->emplace(name, func);
}

BaseSubscriberPtr SubscriberFactory::Create(const std::string& name) {
  const auto iter = _g_subscriber_factory_map_->find(name);
  if (iter != _g_subscriber_factory_map_->end()) {
    auto ptr = iter->second();
    return ptr;
  } else {
    return nullptr;
  }
}

}  // namespace aimrt::module::subscribe
