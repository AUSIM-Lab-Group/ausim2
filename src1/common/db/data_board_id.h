#pragma once

#include <cstring>
#include <string>
#include <string_view>
#include <type_traits>
#include <typeinfo>

namespace aimrte::db {

template <typename _TType_>
std::string_view GetType() {
  static std::string_view name = __PRETTY_FUNCTION__;
  static const std::string_view type_name = [&] {
    auto begin = name.find("with _TType_ = ");
    auto end = name.find("; std::string_view =", begin);

    if (begin != std::string_view::npos && end != std::string_view::npos) {
      return name.substr(begin + 15, end - begin - 15);
    } else {
      return std::string_view("");  // 空的字符串字面量
    }
  }();

  return type_name;
}

template <typename _TType_>
constexpr std::string_view ConstexprGetType() {
  char const* info = __PRETTY_FUNCTION__;
  std::string_view name = [&] {
    // 找各个符号位置
    std::size_t l = strlen(info);
    std::size_t equals = 0, semicolon = 0, end = 0;
    for (std::size_t i = 0; i < l && !end; ++i) {
      switch (info[i]) {
        case '=':
          if (!equals) equals = i;
          break;
        case ';':
          semicolon = i;
          break;
        case ']':
          end = i;
          break;
      }
    }

    if (equals && semicolon && end) {
      return std::string_view(info + equals + 1, semicolon - equals - 1);
    } else {
      return std::string_view("");  // 空的字符串字面量
    }
  }();
  return name;
}

// 特化 GetIdForType 函数以处理字符串类型
template <typename T>
inline constexpr std::size_t GetIdForType(const T& arg) {
  if constexpr (std::is_constructible_v<std::string, decltype(std::declval<T>())>) {
    return std::hash<std::string>{}(std::string(arg));
  } else {
    return std::hash<T>{}(arg) + typeid(arg).hash_code();
  }
}

// 特化 GetIdForType 函数以处理空指针 nullptr
template <>
inline constexpr std::size_t GetIdForType(const std::nullptr_t&) {
  return 0;  // 对于空指针 nullptr 返回固定的哈希值 0
}

template <typename T, typename... Args>
inline std::string GetId(const std::string& tag = "tag::", Args&&... args) {
  std::size_t seed = 0;
  ((seed += GetIdForType(std::forward<Args>(args))), ...);
  return tag + std::string(ConstexprGetType<T>()) + "::" + std::to_string(seed);
}

}  // namespace aimrte::db