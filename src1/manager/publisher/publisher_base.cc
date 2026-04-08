#include <unordered_map>

#include "./publisher_base.h"

namespace aimrt::module::publish {

BasePublisher::BasePublisher() = default;

void BasePublisher::SetCore(aimrt::CoreRef core) { core_ = core; }
void BasePublisher::SetConfig(const PublisherConfig& config) { config_ = config; }

std::string BasePublisher::GetName() const { return config_.name; }
std::string BasePublisher::GetTopic() const { return config_.topic; }
double BasePublisher::GetFrequency() const { return config_.frequency; }
bool BasePublisher::IsEnable() const { return config_.enable; }
bool BasePublisher::IsPrintable() {
  if (config_.print_interval) {
    if (print_interval_ >= config_.print_interval) {
      print_interval_ = 0;
      return true;
    }
  }
  return false;
}
void BasePublisher::PrintCount() {
  if (config_.print_interval) {
    ++print_interval_;
  }
}

using PublisherFactoryMap = std::unordered_map<std::string, PublisherFactory::CreatePublisherFunction>;
static std::shared_ptr<PublisherFactoryMap> _g_publisher_factory_map_{nullptr};

PublisherFactory& PublisherFactory::Instance() {
  static PublisherFactory instance;
  if (!_g_publisher_factory_map_) {
    _g_publisher_factory_map_ = std::make_shared<PublisherFactoryMap>();
  }
  return instance;
}

void PublisherFactory::Register(const std::string& name, CreatePublisherFunction func) {
  if (_g_publisher_factory_map_->contains(name)) {
    return;
  }

  _g_publisher_factory_map_->emplace(name, func);
}

BasePublisherPtr PublisherFactory::Create(const std::string& name) {
  auto iter = _g_publisher_factory_map_->find(name);
  if (iter != _g_publisher_factory_map_->end()) {
    auto ptr = iter->second();
    return ptr;
  }
  return nullptr;
}

}  // namespace aimrt::module::publish
