#pragma once

#include <chrono>
#include <cstdint>
#include <functional>
#include <iostream>
#include <mutex>
#include <queue>
#include <unordered_map>

#include "./waitable.h"

namespace aimrte::db {

template <typename Req, typename Resp>
class SecurityRpcData {
  using Func = std::function<bool(const Req& req, Resp& resp)>;
  enum class RpcType { Request, Response };

  template <typename T>
  class DataWrapper {
   public:
    DataWrapper() = default;
    DataWrapper(const T& data) { SetData(data); }

    void SetData(const T& data) {
      data_ = data;
      sequence_id_ = static_cast<uint64_t>(std::chrono::steady_clock::now().time_since_epoch().count());
    }

    T Data() const { return data_; }
    bool IsValid() const { return sequence_id_ != 0; }
    uint64_t sequence_id() const { return sequence_id_; }

   private:
    T data_;
    uint64_t sequence_id_ = 0;
  };

  class WrapperBase {
   public:
    WrapperBase(bool ok, DataWrapper<Req> req) : ok_(ok), req_(req) {}
    operator bool() const { return ok_; }
    bool operator!() const { return !ok_; }
    bool ok() const { return ok_; }

   protected:
    bool ok_ = false;
    DataWrapper<Req> req_;
  };

  class RequestWrapper : public WrapperBase {
   public:
    RequestWrapper(SecurityRpcData& parent, bool ok, DataWrapper<Req> req) : WrapperBase(ok, req), parent_(parent) {
      if (ok) {
        parent_.LockWaitResponse();
      }
    }
    ~RequestWrapper() { parent_.UnlockWaitResponse(); }
    bool WaitResponse(Resp& resp, std::chrono::milliseconds timeout = std::chrono::milliseconds::max()) {
      if (!WrapperBase::ok()) return false;
      return parent_.WaitResponse(WrapperBase::req_, resp, timeout);
    }

   private:
    SecurityRpcData& parent_;
  };

  class ResponseWrapper : public WrapperBase {
   public:
    ResponseWrapper(SecurityRpcData& parent, bool ok, DataWrapper<Req> req) : WrapperBase(ok, req), parent_(parent) {
      if (ok) {
        parent_.LockWaitRequest();
      }
    }
    ~ResponseWrapper() { parent_.UnlockWaitRequest(); }
    void SetResponse(const Resp& resp) { parent_.SetResponse(WrapperBase::req_, resp); }

   private:
    SecurityRpcData& parent_;
  };

 public:
  SecurityRpcData() = default;
  ~SecurityRpcData() {
    request_waitable_.Notify(false);
    response_waitable_.Notify(false);
  }

  [[nodiscard]] ResponseWrapper WaitRequest(Req& req, std::chrono::milliseconds timeout = std::chrono::milliseconds::max()) {
    if (request_queue_.empty()) {
      LockWaitRequest();
      bool result = request_waitable_.Wait(timeout);
      UnlockWaitRequest();
      if (result == false) {
        return ResponseWrapper(*this, false, DataWrapper<Req>());
      }
    }

    if (request_queue_.empty()) {
      return ResponseWrapper(*this, false, DataWrapper<Req>());
    }

    auto req_wrapper = request_queue_.front();
    request_queue_.pop();

    req = req_wrapper.Data();
    return ResponseWrapper(*this, true, req_wrapper);
  }

  [[nodiscard]] RequestWrapper SetRequest(const Req& req) {
    if (wait_request_mutex_.try_lock()) {
      wait_request_mutex_.unlock();
      return RequestWrapper(*this, false, DataWrapper<Req>());
    }

    auto data = DataWrapper<Req>(req);
    request_queue_.push(data);
    request_waitable_.Notify();
    return RequestWrapper(*this, true, data);
  }

 private:
  void SetResponse(const DataWrapper<Req>& req, const Resp& resp) {
    if (!req.IsValid()) return;
    if (response_map_.find(req.sequence_id()) == response_map_.end()) {
      response_map_[req.sequence_id()] = resp;
      response_waitable_.Notify();
    }
  }

  bool WaitResponse(const DataWrapper<Req>& req, Resp& resp, std::chrono::milliseconds timeout = std::chrono::milliseconds::max()) {
    auto it = response_map_.find(req.sequence_id());
    if (it == response_map_.end()) {
      bool result = response_waitable_.Wait(timeout);
      if (result == false) {
        return false;
      }
      it = response_map_.find(req.sequence_id());
    }

    if (it == response_map_.end()) {
      return false;
    }

    resp = it->second;
    response_map_.erase(it);
    return true;
  }

  void LockWaitRequest() { wait_request_mutex_.lock(); }
  void LockWaitResponse() { wait_response_mutex_.lock(); }
  void UnlockWaitRequest() {
    (void)wait_request_mutex_.try_lock();
    wait_request_mutex_.unlock();
  }
  void UnlockWaitResponse() {
    (void)wait_request_mutex_.try_lock();
    wait_response_mutex_.unlock();
  }

 private:
  Func func_{nullptr};
  Waitable request_waitable_;
  Waitable response_waitable_;
  std::mutex wait_request_mutex_;
  std::mutex wait_response_mutex_;
  std::queue<DataWrapper<Req>> request_queue_;
  std::unordered_map<uint64_t, Resp> response_map_;
};

}  // namespace aimrte::db
