#pragma once

#include <atomic>
#include <functional>
#include <memory>

#include "./data_board_lock.h"

namespace aimrte::db {

// 要求T可被构造成std::atomic<T>
// 在 g++ 13 前，shared_ptr<T> 是不可被std::atomic<T>包裹的
// Concept for types that can be wrapped by std::atomic
template <typename T>
concept AtomicWrappable = (std::is_trivially_copyable_v<T> && std::is_scalar_v<T> && (requires { typename std::atomic<T>; }));

// 要求 T 不是指针，不是引用，不是 std::shared_ptr
template <typename T>
concept NotPointer = (std::is_reference_v<T> && !std::is_same_v<T, std::shared_ptr<T>> && !std::is_pointer_v<T>);

// Primary template of SecurityData
template <typename T, typename = void>
class SecurityData;

class SecurityDataValidity {
 private:
  std::atomic<uint64_t> data_count_ = 0;

 public:
  void Incr() { ++data_count_; }
  void Decr() { --data_count_; }
  bool IsValid() { return data_count_ > 0; }
};

template <typename T>
  requires AtomicWrappable<T> && NotPointer<T>
class SecurityData<T> : public std::atomic<T>, public SecurityDataValidity {
 public:
  SecurityData() = default;
  SecurityData(const SecurityData&) = delete;
  SecurityData& operator=(const SecurityData&) = delete;
  SecurityData(SecurityData&&) = delete;
  SecurityData& operator=(SecurityData&&) = delete;
  ~SecurityData() = default;

  void operator=(const T& data) {
    this->store(data, std::memory_order_relaxed);
    this->Incr();
  }
  void operator=(T&& data) { this->store(data, std::memory_order_relaxed); }
  T operator()() const { return this->load(std::memory_order_relaxed); }
};

enum class Permission { ReadOnly, ReadWrite };

template <typename T, Permission Perm>
class SecurityDataRef;

template <typename T>
class SecurityData<T> : private RWLock, public SecurityDataValidity {
  friend class SecurityDataRef<T, Permission::ReadOnly>;  // 声明 SecurityDataRef 为友元类
  friend class SecurityDataRef<T, Permission::ReadWrite>;

 public:
  SecurityData() = default;
  SecurityData(const SecurityData&) = delete;
  SecurityData& operator=(const SecurityData&) = delete;
  SecurityData(SecurityData&&) = delete;
  SecurityData& operator=(SecurityData&&) = delete;
  ~SecurityData() = default;

  void operator=(const T& data) {
    LockForWrite();
    data_ = data;
    Incr();
    ReleaseWriteLock();
  }

  void operator=(T&& data) {
    LockForWrite();
    data_ = std::move(data);
    Incr();
    ReleaseWriteLock();
  }

  T operator()() const {
    LockForRead();
    T data = data_;
    ReleaseReadLock();
    return data;
  }

 private:
  T data_;
};

template <typename T, Permission Perm>
class SecurityDataRef {
 public:
  explicit SecurityDataRef(std::shared_ptr<SecurityData<T>> data = nullptr) { data_ = data; }
  ~SecurityDataRef() = default;

  // if (this)
  explicit operator bool() const { return data_ != nullptr; }

  void operator=(const T& data) {
    static_assert(Perm == Permission::ReadWrite, "Permission must be ReadWrite");
    data_->operator=(data);
  }
  void operator=(T&& data) {
    static_assert(Perm == Permission::ReadWrite, "Permission must be ReadWrite");
    data_->operator=(std::move(data));
  }

  T operator()() const { return data_->operator()(); }
  T operator*() const { return data_->operator()(); }

  void LockForWrite(std::function<void(T&)> func) const {
    static_assert(Perm == Permission::ReadWrite, "Permission must be ReadWrite");

    if (data_ == nullptr) return;
    if constexpr (AtomicWrappable<T>) {
      // 对原子操作，直接包裹后赋值
      auto data = data_->operator()();
      func(data);
      data_->operator=(std::move(data));
    } else {
      ScopedWriteLock<RWLock> lock(*data_);
      func(data_->data_);
    }
  }

  void LockForRead(std::function<void(const T&)> func) const {
    if (data_ == nullptr) return;
    if constexpr (AtomicWrappable<T>) {
      // 对原子操作，直接包裹后赋值
      auto data = data_->operator()();
      func(data);
    } else {
      // 非原子操作，需要加锁，再操作，此次不需要做权限检查
      ScopedReadLock<RWLock> lock(*data_);
      func(data_->data_);
    }
  }

 private:
  std::shared_ptr<SecurityData<T>> data_ = nullptr;
};

}  // namespace aimrte::db
