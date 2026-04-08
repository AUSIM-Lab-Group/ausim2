#pragma once

#include <filesystem>
#include <functional>
#include <mutex>
#include <unordered_map>

#include "./param_reader.h"
#include "./param_write.h"

// 目的：提供参数管理的接口

namespace param {

class ParamManger {
  using ParamNode = YAML::Node;
  using ParamMap = std::unordered_map<std::string, ParamNode>;  // file : YAML::Node
  using ParamUpdateCallbackList = std::list<std::function<void()>>;
  using ParamNameUpdateCallbackMap = std::unordered_map<std::string, ParamUpdateCallbackList>;     // param_name : callback
  using ParamFileUpdateCallbackMap = std::unordered_map<std::string, ParamNameUpdateCallbackMap>;  // file : (param_name : callback)

 private:
  ParamMap param_map_;
  std::mutex param_mutex_;
  std::mutex param_update_callback_mutex_;
  ParamFileUpdateCallbackMap param_update_callback_map_;

 private:
  ParamManger() = default;
  ~ParamManger() = default;

 public:
  static ParamManger& Instance() {
    static ParamManger instance;
    return instance;
  }

  template <typename T>
  void RegisterParamUpdateCallback(std::function<void(T)> callback, const std::string& file, const std::string param_name = "",
                                   const std::string delimiter = ".") {
    if (param_update_callback_map_.find(file) == param_update_callback_map_.end()) {
      std::unique_lock<std::mutex> lock(param_update_callback_mutex_);
      param_update_callback_map_[file] = ParamNameUpdateCallbackMap();
    }

    std::unique_lock<std::mutex> lock(param_update_callback_mutex_);
    auto& callback_list = param_update_callback_map_[file][param_name];
    auto cb = [this, callback, file, param_name, delimiter]() {
      T value;
      std::unique_lock<std::mutex> lock(param_mutex_);
      auto node = param_map_[file];
      lock.unlock();
      ReadParam(value, node, param_name, delimiter);
      callback(value);
    };
    callback_list.push_back(cb);
  }

  void UpdateParam(const std::string& file) {
    if (param_update_callback_map_.find(file) != param_update_callback_map_.end()) {
      std::unique_lock<std::mutex> lock(param_update_callback_mutex_);
      for (auto& it : param_update_callback_map_[file]) {
        // it.second();
        for (auto& callback : it.second) {
          callback();
        }
      }
    }
  }

  void UpdateParam(const ParamNode& pn, const std::string& file) {
    std::unique_lock<std::mutex> lock(param_mutex_);
    param_map_.erase(file);
    param_map_.insert(std::make_pair(file, pn));
    lock.unlock();

    UpdateParam(file);
  }

  template <typename T>
  bool GetParam(T& value, const ParamNode pn, const std::string param_name = "", const std::string delimiter = ".") {
    return ReadParam(value, pn, param_name, delimiter);
  }

  template <typename T>
  bool GetParam(T& value, const std::string& file, const std::string param_name = "", const std::string delimiter = ".") {
    // 判断文件是否存在
    if (!std::filesystem::exists(file)) {
      // std::cerr << "GetParam: File not found : " << file << std::endl;
      throw std::runtime_error("File not found");
      return false;
    }

    // 如果文件存在，读取文件内容
    std::unique_lock<std::mutex> lock(param_mutex_);
    if (param_map_.find(file) == param_map_.end()) {
      param_map_.insert(std::make_pair(file, YAML::LoadFile(file)));
    }
    ParamNode node = YAML::Clone(param_map_[file]);
    lock.unlock();
    GetParam(value, node, param_name, delimiter);

    return true;
  }

  template <typename T>
  T GetParam(const std::string& file, const std::string param_name = "", const std::string delimiter = ".") {
    T val;
    GetParam(val, file, param_name, delimiter);
    return val;
  }

  template <typename T>
  bool GetParamFormString(T& value, const std::string str, const std::string param_name = "", const std::string delimiter = ".") {
    return ReadParamFormString(value, str, param_name, delimiter);
  }

  template <typename T>
  bool SetParam(const T& value, const std::string& file, const std::string param_name = "", const std::string delimiter = ".") {
    // 判断文件是否存在
    if (!std::filesystem::exists(file)) {
      throw std::runtime_error("File not found");
      return false;
    }

    std::unique_lock<std::mutex> lock(param_mutex_);
    if (param_map_.find(file) == param_map_.end()) {
      param_map_[file] = YAML::LoadFile(file);
    }
    ParamNode node = YAML::Clone(param_map_[file]);
    lock.unlock();

    bool ret = WriteParamToNode(node, value, param_name, delimiter);
    if (!ret) {
      return false;
    }

    std::unique_lock<std::mutex> lock1(param_mutex_);
    param_map_.erase(file);
    param_map_.insert(std::make_pair(file, node));
    lock1.unlock();

    return true;
  }

  template <typename T>
  bool SetParamFormString(const T& value, const std::string str, const std::string param_name = "", const std::string delimiter = ".") {
    ParamNode node = YAML::Load(str);
    bool ret = WriteParamToNode(node, value, param_name, delimiter);
    if (!ret) {
      return false;
    }

    return true;
  }

  ParamMap GetParamMap() {
    std::unique_lock<std::mutex> lock(param_mutex_);
    return param_map_;
  }

  ParamNode GetParamNode(const std::string& file) {
    std::unique_lock<std::mutex> lock(param_mutex_);
    return param_map_[file];
  }

  void ClearParamMap() {
    std::unique_lock<std::mutex> lock(param_mutex_);
    param_map_.clear();
  }
  void ClearParamMap(const std::string& file) {
    std::unique_lock<std::mutex> lock(param_mutex_);
    param_map_.erase(file);
  }
};

}  // namespace param
