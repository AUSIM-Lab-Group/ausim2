
#pragma once

#include <iostream>
#include <string>
#include <vector>
#include "./param_reflection.h"

#include "yaml-cpp/yaml.h"

namespace param {

inline bool SplitString(std::vector<std::string>& tokens, const std::string param_name = "", const std::string delimiter = ".") {
  auto start = param_name.find_first_not_of(delimiter, 0);     // 分割到的字符串的第一个字符
  auto position = param_name.find_first_of(delimiter, start);  // 分隔符的位置
  while (position != std::string::npos || start != std::string::npos) {
    tokens.emplace_back(param_name.substr(start, position - start));
    start = param_name.find_first_not_of(delimiter, position);
    position = param_name.find_first_of(delimiter, start);
  }
  return true;
}

void MergeMap(YAML::Node target, YAML::Node const& source);
void MergeSequences(YAML::Node target, YAML::Node const& source);

inline void MergeNode(YAML::Node target, YAML::Node const& source) {
  switch (source.Type()) {
    case YAML::NodeType::Scalar:
      target = source.Scalar();
      break;
    case YAML::NodeType::Map:
      MergeMap(target, source);
      break;
    case YAML::NodeType::Sequence:
      MergeSequences(target, source);
      break;
    case YAML::NodeType::Null:
      throw std::runtime_error("MergeNode: Null source nodes not supported");
    case YAML::NodeType::Undefined:
      throw std::runtime_error("MergeNode: Undefined source nodes not supported");
  }
}

inline void MergeMap(YAML::Node target, YAML::Node const& source) {
  for (auto const& src : source) {
    MergeNode(target[src.first.Scalar()], src.second);
  }
}

inline void MergeSequences(YAML::Node target, YAML::Node const& source) {
  for (std::size_t i = 0; i != source.size(); ++i) {
    if (i < target.size()) {
      MergeNode(target[i], source[i]);
    } else {
      target.push_back(YAML::Clone(source[i]));
    }
  }
}

inline bool Merge(YAML::Node& target, const YAML::Node& source) {
  if (!target.IsDefined()) {
    target = YAML::Clone(source);
    return true;
  }
  try {
    MergeNode(target, source);
  } catch (std::exception& e) {
    std::cerr << "Merge error: " << e.what() << std::endl;
    return false;
  }
  return true;
}

//-------------------------------------------------------------------------------------------------------------//
template <refletable T>
void ToString(T& p, std::string& s, std::size_t last_indentation);

template <sequence_container T>
void ToString(T& p, std::string& s, std::size_t last_indentation, const std::string& name) {
  std::size_t ind = last_indentation;
  constexpr std::size_t indentation = 2;
  using element_type = std::decay_t<decltype(p)>::value_type;

  // 如果还是容器类型
  if constexpr (is_sequence_container<element_type>::value) {
    int count = 0;
    ind += indentation;
    s.append("\n");
    for (auto& item : p) {
      s.append(ind, ' ');
      s.append(name + "[" + std::to_string(count) + "]: ");
      ind += indentation;
      ToString(item, s, ind, name);
      ind -= indentation;
      count++;
    }
    ind -= indentation;
  }

  if constexpr (is_reflection<element_type>::value) {
    int count = 0;
    ind += indentation;
    s.append("\n");
    for (auto& item : p) {
      s.append(ind, ' ');
      s.append(name + "[" + std::to_string(count) + "]:\n");
      ind += indentation;
      ToString(item, s, ind);
      ind -= indentation;
      count++;
    }
    ind -= indentation;
  }

  // 如果是string类型
  if constexpr (std::is_same_v<element_type, std::string>) {
    s.append("[");
    for (auto& item : p) {
      s.append("\"");
      s.append(item);
      s.append("\"");
      s.append(", ");
    }
    if (p.size()) {
      s.pop_back();
      s.pop_back();
    }
    s.append("]");
    s.append("\n");
  }

  // 如果是基础类型 int float double bool
  if constexpr (std::is_arithmetic_v<element_type>) {
    s.append("[");
    for (auto item : p) {
      s.append(std::to_string(item));
      s.append(", ");
    }
    if (p.size()) {
      s.pop_back();
      s.pop_back();
    }
    s.append("]");
    s.append("\n");
  }
}

template <reflet_is_map T>
void ToString(T& p, std::string& s, std::size_t last_indentation, const std::string& name) {
  std::size_t ind = last_indentation;
  s.append(ind, ' ');
  s.append("[\n");
  for (auto& item : p) {
    // s.append("\"");
    s.append(ind + 2, ' ');
    s.append(item.first);
    // s.append("\"");
    s.append(": ");
    if constexpr (is_sequence_container<std::decay_t<decltype(item.second)>>::value) {
      ToString(item.second, s, ind, name);
      s.pop_back();
      s.append("\n");
    } else if constexpr (is_reflection<std::decay_t<decltype(item.second)>>::value) {
      s.append("\n");
      ToString(item.second, s, ind + 4);
      s.pop_back();
      s.append("\n");
    } else {
      // 如果是string类型
      if constexpr (std::is_same_v<std::decay_t<decltype(item.second)>, std::string>) {
        s.append("\"");
        s.append(item.second);
        s.append("\"");
      } else if constexpr (std::is_arithmetic_v<std::decay_t<decltype(item.second)>>) {
        s.append(std::to_string(item.second));
      }
      s.append("\n");
    }
  }
  if (p.size()) {
    s.pop_back();
  }
  s.append("\n");
  s.append(ind, ' ');
  s.append("]\n");
}

template <refletable T>
void ToString(T& p, std::string& s, std::size_t last_indentation) {
  constexpr std::size_t indentation = 2;
  param::for_each(std::forward<T>(p), [&p, &s, indentation, last_indentation](const auto& v, auto i) {
    std::size_t ind = last_indentation;
    using M = decltype(param_reflect_members(std::forward<T>(p)));
    constexpr auto Idx = decltype(i)::value;
    constexpr auto Count = M::value();
    static_assert(Idx < Count);

    constexpr auto name = param::get_name<decltype(p), decltype(i)::value>();
    s.append(ind, ' ');
    s.append(name);
    s.append(": ");
    // 如果是容器类型，递归
    if constexpr (is_sequence_container<std::decay_t<decltype(p.*v)>>::value) {
      ToString(p.*v, s, ind, std::string(name));
    } else if constexpr (is_reflection<std::decay_t<decltype(p.*v)>>::value) {
      s.append("\n");
      ind += indentation;
      ToString(p.*v, s, ind);
      ind -= indentation;
    } else if constexpr (reflection_is_map<std::decay_t<decltype(p.*v)>>::value) {
      s.append("\n");
      ind += indentation;
      ToString(p.*v, s, ind, std::string(name));
      ind -= indentation;
    } else {
      // 如果是string类型
      if constexpr (std::is_same_v<std::decay_t<decltype(p.*v)>, std::string>) {
        s.append("\"");
        s.append(p.*v);
        s.append("\"");
      } else if constexpr (std::is_arithmetic_v<std::decay_t<decltype(p.*v)>>) {
        s.append(std::to_string(p.*v));
      }
      s.append("\n");
    }
  });
}

template <refletable T>
std::string ToString(T& p, std::size_t indentation = 2) {
  std::string s;
  ToString(p, s, indentation);
  return s;
}

//-------------------------------------------------------------------------------------------------------------//

template <refletable T>
bool ToYaml(YAML::Node& node, const T& p);

template <sequence_container T>
bool ToYaml(YAML::Node& node, const T& p) {
  // 获取容器的元素类型
  using element_type = typename T::value_type;
  // 如果容器的元素是可反射的
  if constexpr (is_reflection<element_type>::value) {
    // 将容器的元素写入到node中
    for (auto& item : p) {
      YAML::Node temp;
      ToYaml(temp, item);
      node.push_back(temp);
    }
  } else {
    // 将容器的元素写入到node中
    for (auto& item : p) {
      if constexpr (std::is_same_v<element_type, double>) {
        std::string temp_str = std::to_string(item);
        if (temp_str.find('.') == std::string::npos) {
          temp_str += ".0";
        }
        node.push_back(temp_str);
      } else {
        // 属于其他类型
        node.push_back(item);
      }
    }
  }
  return true;
}

template <refletable T>
bool ToYaml(YAML::Node& node, const T& p) {
  for_each(std::forward<const T>(p), [&node, p = p](auto& v, auto i) {
    using M = decltype(param_reflect_members(std::forward<const T>(p)));
    constexpr auto Idx = decltype(i)::value;
    constexpr auto Count = M::value();
    static_assert(Idx < Count);
    constexpr std::string_view name = get_name<decltype(p), decltype(i)::value>();
    std::string name_str(name);
    YAML::Node temp;
    if constexpr (is_sequence_container<std::decay_t<decltype(p.*v)>>::value) {
      using element_type = std::decay_t<decltype(p.*v)>::value_type;

      if constexpr ((is_reflection<element_type>::value)) {
        // 属于容器，且容器的元素是可反射的，或者容器内容类型依旧是容器
        ToYaml(temp, p.*v);
      } else {
        // 属于容器，但容器的元素不是可反射的
        // 将容器的元素写入到temp中
        if constexpr (std::is_same_v<element_type, double>) {
          for (auto& item : p.*v) {
            std::string temp_str = std::to_string(item);
            if (temp_str.find('.') == std::string::npos) {
              temp_str += ".0";
            }
            temp.push_back(temp_str);
          }
        } else {
          // 属于其他类型
          temp = (p.*v);
        }
      }
    } else if constexpr (is_reflection<std::decay_t<decltype(p.*v)>>::value) {
      // 属于可反射的类型
      ToYaml(temp, p.*v);
    } else {
      // 属于不可反射的类型
      if constexpr (std::is_same_v<std::decay_t<decltype(p.*v)>, double>) {
        std::string temp_str = std::to_string(p.*v);
        if (temp_str.find('.') == std::string::npos) {
          temp_str += ".0";
        }
        temp = temp_str;
      } else {
        // 属于其他类型
        temp = (p.*v);
      }
    }
    node[name_str] = temp;
  });

  return true;
}

template <sequence_container T>
YAML::Node ToYaml(const T& p) {
  YAML::Node node;
  ToYaml(node, p);
  return node;
}

template <reflet_is_map T>
YAML::Node ToYaml(const T& p) {
  YAML::Node node;
  ToYaml(node, p);
  return node;
}

template <refletable T>
YAML::Node ToYaml(const T& p) {
  YAML::Node node;
  ToYaml(node, p);
  return node;
}

//-------------------------------------------------------------------------------------------------------------//
template <refletable T>
std::string ToYamlString(const T& p) {
  YAML::Node node;
  ToYaml(node, p);
  return YAML::Dump(node);
}

}  // namespace param
