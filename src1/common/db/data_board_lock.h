#pragma once

#include <atomic>

namespace aimrte::db {

class RWLock {
 public:
  RWLock() = default;
  RWLock(const RWLock&) = delete;
  RWLock(RWLock&&) = delete;
  virtual ~RWLock() = default;

  void LockForWrite() const {
    int expected = ConstLockFree;
    while (!lock_.compare_exchange_weak(expected, ConstLockWrite, std::memory_order_acquire, std::memory_order_relaxed)) {
      expected = ConstLockFree;
    }
  }

  void LockForRead() const {
    int lock = lock_.load(std::memory_order_relaxed);
    while (lock < ConstLockFree || !lock_.compare_exchange_weak(lock, lock + 1, std::memory_order_acquire, std::memory_order_relaxed)) {
      lock = lock_.load(std::memory_order_relaxed);
    }
  }

  void ReleaseWriteLock() const {
    if (lock_.load(std::memory_order_acquire) != ConstLockWrite) {
      return;
    }
    lock_.store(ConstLockFree, std::memory_order_release);
  }

  void ReleaseReadLock() const {
    if (lock_.load(std::memory_order_acquire) <= ConstLockFree) {
      return;
    }
    lock_.fetch_sub(1, std::memory_order_release);
  }

  bool TryLockForWrite() const {
    int expected = ConstLockFree;
    return lock_.compare_exchange_strong(expected, ConstLockWrite, std::memory_order_acquire, std::memory_order_relaxed);
  }
  bool TryLockForRead() const {
    int lock = lock_.load(std::memory_order_relaxed);
    return lock >= ConstLockFree && lock_.compare_exchange_strong(lock, lock + 1, std::memory_order_acquire, std::memory_order_relaxed);
  }

 private:
  constexpr static int ConstLockFree = 0;
  constexpr static int ConstLockWrite = -1;
  mutable std::atomic<int> lock_{0};
};

// RAII 包装类
template <class TLock>
class ScopedWriteLock {
 public:
  explicit ScopedWriteLock(const TLock& lock) : lock_(lock) { lock_.LockForWrite(); }
  ~ScopedWriteLock() { lock_.ReleaseWriteLock(); }

 private:
  const RWLock& lock_;
};

// RAII 包装类
template <class TLock>
class ScopedReadLock {
 public:
  explicit ScopedReadLock(const TLock& lock) : lock_(lock) { lock_.LockForRead(); }
  ~ScopedReadLock() { lock_.ReleaseReadLock(); }

 private:
  const TLock& lock_;
};

}  // namespace aimrte::db
