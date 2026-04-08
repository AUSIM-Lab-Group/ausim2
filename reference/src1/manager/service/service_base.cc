#include <unordered_map>

#include "./service_base.h"

namespace aimrt::module::service {

ServiceImpl::ServiceImpl(BaseServicePtr service) : service_(std::move(service)) {}

void ServiceImpl::SetCore(aimrt::CoreRef core) { core_ = core; }
void ServiceImpl::SetConfig(const ServiceConfig& config) { config_ = config; }

std::string ServiceImpl::GetName() const { return config_.name; }
bool ServiceImpl::IsEnable() const { return config_.enable; }
bool ServiceImpl::IsRunning() const { return running_; }
bool ServiceImpl::IsPrintable() const { return config_.debug_print; }

bool ServiceImpl::Init() {
  if (!service_) return false;
  if (!core_) return false;

  if (!config_.executor.empty()) {
    executor_ = core_.GetExecutorManager().GetExecutor(config_.executor);
    AIMRT_CHECK_ERROR_THROW(executor_, "Get executor for name '{}' failed.", config_.executor);
  }

  service_->RegisterFilter([this](aimrt::rpc::ContextRef ctx, const void* req_ptr, void* rsp_ptr,
                                  const aimrt::rpc::CoRpcHandle& next) -> aimrt::co::Task<aimrt::rpc::Status> {
    if (!IsRunning()) co_return aimrt::rpc::Status{aimrt_rpc_status_code_t::AIMRT_RPC_STATUS_SVR_HANDLE_FAILED};

    if (executor_) {
      aimrt::co::AimRTScheduler scheduler(executor_);
      co_await aimrt::co::Schedule(scheduler);
    }

    if (IsPrintable()) {
      auto json = aimrt::Pb2CompactJson(*static_cast<const google::protobuf::Message*>(req_ptr));
      if (json.size() > 2048) json = json.substr(0, 2048) + " ...";
      AIMRT_INFO("svr [{}] get new rpc call. req: {}", GetName(), json);
    }

    const auto& status = co_await next(ctx, req_ptr, rsp_ptr);

    if (IsPrintable()) {
      auto json = aimrt::Pb2CompactJson(*static_cast<const google::protobuf::Message*>(rsp_ptr));
      if (json.size() > 2048) json = json.substr(0, 2048) + " ...";
      AIMRT_INFO("svr [{}] handle rpc completed, status: {}, rsp: {}", GetName(), status.ToString(), json);
    }

    co_return status;
  });

  bool ret = core_.GetRpcHandle().RegisterService(service_.get());
  AIMRT_CHECK_ERROR_THROW(ret, "Register service {} failed.", GetName());

  return ret;
}

void ServiceImpl::Start() { running_ = true; }
void ServiceImpl::Shutdown() { running_ = false; }

using ServiceFunctionMap = std::unordered_map<std::string, ServiceFactory::CreateServiceFunction>;
static std::shared_ptr<ServiceFunctionMap> _g_service_factory_map_{nullptr};

ServiceFactory& ServiceFactory::Instance() {
  static ServiceFactory instance;
  if (!_g_service_factory_map_) {
    _g_service_factory_map_ = std::make_shared<ServiceFunctionMap>();
  }
  return instance;
}

void ServiceFactory::Register(const std::string& name, CreateServiceFunction func) {
  if (_g_service_factory_map_->contains(name)) {
    return;
  }

  _g_service_factory_map_->emplace(name, func);
}

ServiceImplPtr ServiceFactory::Create(const std::string& name) {
  const auto iter = _g_service_factory_map_->find(name);
  if (iter != _g_service_factory_map_->end()) {
    auto ptr = iter->second();
    return ptr;
  }
  return nullptr;
}

}  // namespace aimrt::module::service
