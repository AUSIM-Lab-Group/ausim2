#pragma once

#include <unordered_map>

#include "./client_base.h"
#include "yaml-cpp/yaml.h"

#include "aima/sim/manager/executor/cycle_free_executor.h"

namespace aimrt::module::client {

class ClientManager {
 public:
  ClientManager(aimrt::CoreRef core);
  ~ClientManager();

  void Init();

  void Start() const;

  void Shutdown() const;

 private:
  aimrt::CoreRef core_;
  using PublisherExecutorPtr = std::shared_ptr<executor::CycleFreeExecutor>;
  std::unordered_map<std::string, PublisherExecutorPtr> client_executors_;
};

}  // namespace aimrt::module::client
