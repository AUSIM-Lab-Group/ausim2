#pragma once

#include <optional>
#include <queue>

#include "./data_board.h"
#include "./waitable.h"

namespace aimrte::db {

// WaitableQueue 类
template <typename T>
class WaitableQueue : public Waitable, public std::queue<T> {
 public:
  WaitableQueue() = default;
  virtual ~WaitableQueue() = default;

  void Push(const T& item) {
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      this->push(item);
    }
    Notify();
  }

  [[nodiscard]] std::optional<T> WaitAndPop(std::chrono::milliseconds timeout = std::chrono::milliseconds::max()) {
    if (!Wait(timeout)) {
      return std::nullopt;
    }

    std::lock_guard<std::mutex> lock(queue_mutex_);
    if (this->empty()) {
      return std::nullopt;
    }
    T item = std::move(this->front());
    this->pop();
    return item;
  }

  [[nodiscard]] std::optional<T> TryPop() {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    if (this->empty()) {
      return std::nullopt;
    }
    T item = std::move(this->front());
    this->pop();
    return item;
  }

  void Clear() {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    while (!this->empty()) {
      this->pop();
    }
  }

 private:
  std::mutex queue_mutex_;
};

// WaitableMap 类
template <typename K, typename V>
class WaitableMap : public Waitable, public std::unordered_map<K, V> {
 public:
  WaitableMap() = default;
  virtual ~WaitableMap() = default;

  void Set(const K& key, const V& value) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      (*this)[key] = value;
    }
    Notify();
  }

  [[nodiscard]] std::optional<V> WaitAndPop(const K& key, std::chrono::milliseconds timeout = std::chrono::milliseconds::max()) {
    if (!Wait(timeout)) {
      return std::nullopt;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (!contains(key)) {
      return std::nullopt;
    }
    V value = std::move((*this)[key]);
    erase(key);
    return value;
  }

  [[nodiscard]] std::optional<V> TryPop(const K& key) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!contains(key)) {
      return std::nullopt;
    }
    V value = std::move((*this)[key]);
    erase(key);
    return value;
  }

  void Clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    this->clear();
  }

  void Clear(const K& key) {
    std::lock_guard<std::mutex> lock(mutex_);
    erase(key);
  }

 private:
  std::mutex mutex_;
};

// 别名定义
template <typename T>
using SecurityWaitableQueue = WaitableQueue<T>;

template <typename K, typename V>
using SecurityWaitableMap = WaitableMap<K, V>;

template <typename T, typename... Args>
[[nodiscard]] std::shared_ptr<SecurityWaitableQueue<T>> GetWaitableQueue(Args&&... args) {
  return aimrte::db::DataBoard().GetCustomized<SecurityWaitableQueue<T>>("waitable::", std::forward<Args>(args)...);
}

template <typename K, typename V, typename... Args>
[[nodiscard]] std::shared_ptr<SecurityWaitableMap<K, V>> GetWaitableMap(Args&&... args) {
  return aimrte::db::DataBoard().GetCustomized<SecurityWaitableMap<K, V>>("waitable::", std::forward<Args>(args)...);
}

}  // namespace aimrte::db
