#include <unordered_map>

#include "./client_base.h"

namespace aimrt::module::client {

void BaseClient::SetCore(aimrt::CoreRef core) { core_ = core; }
void BaseClient::SetConfig(const ClientConfig& config) {
  config_ = config;
  // SelfConfig();
}

std::string BaseClient::GetName() const { return config_.name; }
bool BaseClient::IsEnable() const { return config_.enable; }

bool BaseClient::IsPrintable() {
  if (config_.print_interval) {
    if (print_interval_ >= config_.print_interval) {
      print_interval_ = 0;
      return true;
    }
  }
  return false;
}

void BaseClient::PrintCount() {
  if (config_.print_interval) {
    ++print_interval_;
  }
}

using ClinetFunctionMap = std::unordered_map<std::string, ClientFactory::CreateClientFunction>;
static std::shared_ptr<ClinetFunctionMap> _g_client_factory_map_{nullptr};

ClientFactory& ClientFactory::Instance() {
  static ClientFactory instance;
  if (!_g_client_factory_map_) {
    _g_client_factory_map_ = std::make_shared<ClinetFunctionMap>();
  }
  return instance;
}

void ClientFactory::Register(const std::string& name, CreateClientFunction func) {
  if (_g_client_factory_map_->contains(name)) {
    return;
  }
  _g_client_factory_map_->emplace(name, func);
}

BaseClientPtr ClientFactory::Create(const std::string& name) {
  const auto iter = _g_client_factory_map_->find(name);
  if (iter != _g_client_factory_map_->end()) {
    auto ptr = iter->second();
    return ptr;
  }
  return nullptr;
}

}  // namespace aimrt::module::client
