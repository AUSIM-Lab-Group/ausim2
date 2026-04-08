#pragma once

#include <time.h>
#include <cstdint>

#include "./global.h"

#include "aimrt_module_cpp_interface/co/aimrt_context.h"
#include "aimrt_module_cpp_interface/co/schedule.h"
#include "aimrt_module_cpp_interface/co/sync_wait.h"
#include "aimrt_module_cpp_interface/co/task.h"
#include "aimrt_module_cpp_interface/core.h"
#include "aimrt_module_cpp_interface/rpc/rpc_handle.h"
#include "aimrt_module_cpp_interface/rpc/rpc_status.h"
#include "aimrt_module_protobuf_interface/channel/protobuf_channel.h"
#include "aimrt_module_protobuf_interface/util/protobuf_tools.h"
#include "aimrt_module_ros2_interface/channel/ros2_channel.h"

namespace aimrt::helper {

template <class MsgType>
aimrt::channel::PublisherRef GetPublisher(aimrt::CoreRef core, const std::string& topic_name) {
  const auto publisher = core.GetChannelHandle().GetPublisher(topic_name);
  AIMRT_CHECK_ERROR_THROW(publisher, "Get publisher for topic '{}' failed.", topic_name);
  const bool ret = aimrt::channel::RegisterPublishType<MsgType>(publisher);
  AIMRT_CHECK_ERROR_THROW(ret, "Register publishType failed.");
  return publisher;
}

template <class Proxy>
std::shared_ptr<Proxy> GetProxy(aimrt::CoreRef core) {
  try {
    // Get rpc handle
    auto rpc_handle = core.GetRpcHandle();
    AIMRT_CHECK_ERROR_THROW(rpc_handle, "Get rpc handle failed.");

    // Register rpc client
    bool ret = aimrt::rpc::RegisterClientFunc<Proxy>(rpc_handle);
    AIMRT_CHECK_ERROR_THROW(ret, "Register client failed.");

    return std::make_shared<Proxy>(rpc_handle);

  } catch (const std::exception& e) {
    std::cerr << "GetProxy failed: " << e.what() << std::endl;
  }
  return nullptr;
}

template <class MsgType>
bool SubscriberCo(aimrt::CoreRef core, const std::string& topic_name, const std::string& executor_name,
                  aimrt::util::Function<co::Task<void>(const MsgType&)>&& callback) {
  auto subscriber = core.GetChannelHandle().GetSubscriber(topic_name);
  AIMRT_CHECK_ERROR_THROW(subscriber, "Get subscriber for topic '{}' failed.", topic_name);
  auto ret = aimrt::channel::SubscribeCo<MsgType>(subscriber, [=](const MsgType& msg) -> co::Task<void> {
    static auto executor = core.GetExecutorManager().GetExecutor(executor_name);
    aimrt::co::AimRTScheduler scheduler(executor);
    co_await aimrt::co::Schedule(scheduler);
    // PrintCount
    co_await callback(msg);
    co_return;
  });
  AIMRT_CHECK_ERROR_THROW(ret, "Subscribe failed.");

  return ret;
}

}  // namespace aimrt::helper
