#pragma once

#include <any>
#include <cstddef>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <unordered_map>
#include "./data_board_id.h"
#include "./security_data.h"
#include "./sercurity_rpc_data.h"

namespace aimrte::db {

class DataBoardManager {
 private:
  DataBoardManager() = default;
  DataBoardManager(const DataBoardManager&) = delete;
  DataBoardManager& operator=(const DataBoardManager&) = delete;
  DataBoardManager(DataBoardManager&&) = delete;
  DataBoardManager& operator=(DataBoardManager&&) = delete;
  ~DataBoardManager() = default;

 public:
  static DataBoardManager& Instance() {
    static DataBoardManager instance;
    return instance;
  }

  template <typename T, typename... Args>
  [[nodiscard]] std::shared_ptr<T> GetCustomized(const std::string& tag_str, Args&&... args) {
    std::string key = GetId<T>(tag_str, std::forward<Args>(args)...);
    std::shared_ptr<T> data;
    if (GetOrNull(data, key)) {
      return data;
    }

    // 判断T是否存在无参构造函数
    if constexpr (std::is_default_constructible_v<T>) {
      data = std::make_shared<T>();
      Save(data, key);
      return data;
    }
    return nullptr;
  }

  template <typename T, typename... Args>
  [[nodiscard]] std::shared_ptr<T> GetUnsafe(Args&&... args) {
    return GetCustomized<T>("unsafe::", std::forward<Args>(args)...);
  }

  template <typename T, typename Tag, typename... ConstructorArgs>
  [[nodiscard]] std::shared_ptr<T> GetAndConstructorWitTagStr(const std::string& tag_str, Tag&& tag, ConstructorArgs&&... args) {
    std::string key = GetId<T>(tag_str, std::forward<Tag>(tag));
    std::shared_ptr<T> data;
    if (GetOrNull(data, key)) {
      return data;
    }
    data = std::make_shared<T>(std::forward<ConstructorArgs>(args)...);
    Save(data, key);
    return data;
  }

  template <typename T, typename Tag, typename... ConstructorArgs>
  [[nodiscard]] std::shared_ptr<T> GetAndConstructorWitTag(Tag&& tag, ConstructorArgs&&... args) {
    return GetAndConstructorWitTagStr<T, Tag>("constructor::", std::forward<Tag>(tag), std::forward<ConstructorArgs>(args)...);
  }

  template <typename T, typename... ConstructorArgs>
  [[nodiscard]] std::shared_ptr<T> GetAndConstructor(ConstructorArgs&&... args) {
    return GetAndConstructorWitTag<T, std::nullptr_t>(std::nullptr_t{}, std::forward<ConstructorArgs>(args)...);
  }

  template <typename T, typename... Args>
  void SetUnsafe(std::shared_ptr<T>& data, Args&&... args) {
    std::string key = GetId<T>("unsafe::", std::forward<Args>(args)...);
    Save(data, key);
  }

  template <typename T, Permission P = Permission::ReadOnly, typename... Args>
  [[nodiscard]] SecurityDataRef<T, P> Get(Args&&... args) {
    std::string key = GetId<T>("safe::", std::forward<Args>(args)...);
    std::shared_ptr<SecurityData<T>> data;
    if (GetOrNull(data, key)) {
      return SecurityDataRef<T, P>(data);
    }
    data = std::make_shared<SecurityData<T>>();
    Save(data, key);
    return SecurityDataRef<T, P>(data);
  }

  template <typename Req, typename Resp, typename... Args>
  [[nodiscard]] std::shared_ptr<SecurityRpcData<Req, Resp>> GetRpc(Args&&... args) {
    std::string key = GetId<SecurityRpcData<Req, Resp>>("rpc::", std::forward<Args>(args)...);
    std::shared_ptr<SecurityRpcData<Req, Resp>> data;
    if (GetOrNull(data, key)) {
      return data;
    }
    data = std::make_shared<SecurityRpcData<Req, Resp>>();
    Save(data, key);
    return data;
  }

  template <typename T, typename... Args>
  [[nodiscard]] SecurityDataRef<T, Permission::ReadOnly> Read(Args&&... args) {
    return Get<T, Permission::ReadOnly>(std::forward<Args>(args)...);
  }

  template <typename T, typename... Args>
  [[nodiscard]] SecurityDataRef<T, Permission::ReadWrite> Write(Args&&... args) {
    return Get<T, Permission::ReadWrite>(std::forward<Args>(args)...);
  }

  // 打印所有的数据
  std::string Info() {
    std::stringstream ss;
    uint32_t count = 0;
    for (const auto& pair : data_board_map_) {
      ss << "data[" << count++ << "]: " << pair.first << std::endl;
    }
    return ss.str();
  }

 private:
  template <typename T, typename... Args>
  bool GetOrNull(T& data, const std::string& key) {
    if (auto it = data_board_map_.find(key); it != data_board_map_.end()) {
      try {
        data = std::any_cast<T>(it->second);
        return true;
      } catch (const std::bad_any_cast& e) {
        std::throw_with_nested(std::runtime_error("Any cast error: " + std::string(e.what())));
      }
    }
    return false;
  }

  template <typename T>
  void Save(const T& data, const std::string& key) {
    std::lock_guard<std::mutex> lock(data_board_mutex_);
    if (auto it = data_board_map_.find(key); it != data_board_map_.end()) {
      data_board_map_.erase(it);
    }
    data_board_map_.emplace(key, std::any(data));
  }

 private:
  std::mutex data_board_mutex_;
  std::unordered_map<std::string, std::any> data_board_map_;
};

// 别名一下
auto inline DataBoard = &DataBoardManager::Instance;

}  // namespace aimrte::db
