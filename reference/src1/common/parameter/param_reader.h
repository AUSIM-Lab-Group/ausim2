#pragma once

#include "yaml-cpp/yaml.h"

#include <string>
#include <vector>

#include "./param_reflection.h"
#include "./param_util.h"

namespace param {

template <refletable T>
bool ReadParam(T& p, const YAML::Node node);

template <sequence_container T>
bool ReadParam(T& p, const YAML::Node node) {
  // 获取容器的元素类型
  using element_type = typename T::value_type;

  for (auto& item : node) {
    // 判断 node 是否存在 key，可能是匿名的
    std::string key = item.first.IsDefined() ? item.first.as<std::string>() : "";
    if (key.empty()) {
      element_type element;
      ReadParam(element, item);
      p.emplace_back(element);
    } else {
      auto& temp_item = node[key];  // 拿到item节点
      element_type element;
      ReadParam(element, temp_item);
      p.emplace_back(element);
    }
  }

  return true;
}

template <reflet_is_map T>
bool ReadParam(T& p, const YAML::Node node) {
  using Key = typename T::key_type;
  using Value = typename T::mapped_type;
  for (const auto& item : node) {
    Key key = item.first.as<Key>();
    Value value;
    if constexpr ((is_reflection<Value>::value)) {
      ReadParam(value, item.second);
    } else if constexpr ((is_sequence_container<Value>::value)) {
      using element_type = typename Value::value_type;
      if constexpr ((is_reflection<element_type>::value)) {
        element_type element;
        ReadParam(element, item.second);
        value = element;
      } else {
        value = item.second.as<Value>();
      }
    } else {
      value = item.second.as<Value>();
    }
    p[key] = value;
  }

  return true;
}

template <refletable T>
bool ReadParam(T& p, const YAML::Node node) {
  for_each(std::forward<T>(p), [&p, node](auto& v, auto i) {
    using M = decltype(param_reflect_members(std::forward<T>(p)));
    constexpr auto Idx = decltype(i)::value;
    constexpr auto Count = M::value();
    static_assert(Idx < Count);
    constexpr char none[] = "\033[0m";
    // constexpr char red[] = "\033[1;31m";
    constexpr char yellow[] = "\033[1;33m";
    constexpr std::string_view name = get_name<decltype(p), decltype(i)::value>();
    std::string name_str(name);
    // 判断是否为map数据类型
    if constexpr (reflection_is_map<std::decay_t<decltype(p.*v)>>::value) {
      ReadParam(p.*v, node);
    } else {
      if (node[name_str].IsDefined()) {
        if constexpr (is_sequence_container<std::decay_t<decltype(p.*v)>>::value) {
          using element_type = std::decay_t<decltype(p.*v)>::value_type;

          if constexpr ((is_reflection<element_type>::value)) {
            YAML::Node temp = node[name_str];
            // 属于容器，且容器的元素是可反射的，或者它依旧是属于容器
            ReadParam(p.*v, temp);
          } else {
            // 属于容器，且容器的元素不可反射，直接赋值
            // 打印类型信息
            if (node[name_str].size() == 0) {
              // std::cerr << red << "[error] ReadParam error : " << name_str << " is not defined" << none << std::endl;
            } else {
              p.*v = node[name_str].as<std::decay_t<decltype(p.*v)>>();
            }
          }
        } else if constexpr (is_reflection<std::decay_t<decltype(p.*v)>>::value) {
          //   如果是可反射的结构体，递归
          YAML::Node temp = node[name_str];
          ReadParam(p.*v, temp);
        } else {
          if constexpr (std::is_same<std::decay_t<decltype(p.*v)>, float>::value) {
            //   浮点数类型，需要特殊处理，因为yaml-cpp默认将浮点数解析为double类型
            std::string str = node[name_str].as<std::string>();
            p.*v = std::stof(str);
            std::cout << yellow << "[warning] ReadParam warning : '" << name_str << "' is float type, value is " << str
                      << ", but yaml-cpp parse it as double type, so we convert it to float type，but recommended to use double type" << none
                      << std::endl;
          } else {
            //   不可反射的类型，直接赋值
            p.*v = node[name_str].as<std::decay_t<decltype(p.*v)>>();
          }
        }
      } else {
        // 判断是否为map数据类型
        if constexpr (reflection_is_map<std::decay_t<decltype(p.*v)>>::value) {
          ReadParam(p.*v, node);
        } else {
          // std::cerr << red << "[error] ReadParam error : " << name_str << " is not defined" << none << std::endl;
        }
      }
    }
  });
  return true;
}

template <class T>
bool ReadParam(T& param, const YAML::Node pn, const std::string param_name, const std::string delimiter) {
  bool ret = false;
  try {
    std::vector<std::string> tokens;
    YAML::Node node = YAML::Clone(pn);
    SplitString(tokens, param_name, delimiter);
    for (auto& key : tokens) {
      node = node[key];
    }

    ret = ReadParam(param, node);
  } catch (const std::exception& e) {
    // std::cerr << "ReadParam error : " << e.what() << std::endl;
  }
  return ret;
}

template <class T>
bool ReadParam(T& param, const std::string yaml_file, const std::string param_name = "", const std::string delimiter = ".") {
  try {
    YAML::Node node = YAML::LoadFile(yaml_file);

    return ReadParam(param, node, param_name, delimiter);
  } catch (const std::exception& e) {
    // std::cerr << "ReadParam error : " << e.what() << std::endl;
  }
  return false;
}

template <class T>
bool ReadParamFormString(T& param, const std::string str, const std::string param_name = "", const std::string delimiter = ".") {
  try {
    YAML::Node node = YAML::Load(str);
    return ReadParam(param, node, param_name, delimiter);
  } catch (const std::exception& e) {
    // std::cerr << "ReadParam error : " << e.what() << std::endl;
  }
  return false;
}

}  // namespace param
