#pragma once

#include <chrono>
#include <condition_variable>
#include <iostream>
#include <mutex>

namespace aimrte::db {

class Waitable {
 public:
  // 默认构造函数
  Waitable() : condition_(false) {}
  ~Waitable() { Notify(false); }

  // 设置条件为true，并唤醒等待的线程
  void Notify(bool condition = true) {
    std::unique_lock<std::mutex> lock(mutex_);
    condition_ = condition;
    condition_variable_.notify_all();
  }

  // 等待条件为true，无限期等待
  [[nodiscard]] bool Wait() {
    std::unique_lock<std::mutex> lock(mutex_);
    condition_variable_.wait(lock, [this] { return condition_; });
    condition_ = false;
    return true;
  }

  // 等待条件为true，超时返回false
  [[nodiscard]] bool Wait(std::chrono::milliseconds timeout) {
    if (timeout == std::chrono::milliseconds::max()) return Wait();

    std::unique_lock<std::mutex> lock(mutex_);
    if (!condition_variable_.wait_for(lock, timeout, [this] { return condition_; })) {
      return false;  // 超时
    }
    condition_ = false;
    return true;
  }

 private:
  std::mutex mutex_;
  std::condition_variable condition_variable_;
  bool condition_ = false;
};

}  // namespace aimrte::db
