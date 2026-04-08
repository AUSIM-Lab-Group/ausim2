#pragma once

#include <iomanip>
#include <sstream>
#include "aima/sim/data/joint.hpp"

namespace sim::joint::tools {

#define JOINT_TOOLS_GET_INFO_IMPL(func, field) \
  template <typename T>                        \
  inline std::string func(T &info) {           \
    std::stringstream ss;                      \
    for (auto &c : info.joints) {              \
      ss << c.field << "  ";                   \
    }                                          \
    return ss.str();                           \
  }

#define JOINT_TOOLS_GET_NAME_INFO_IMPL(func, field)                                        \
  template <typename T>                                                                    \
  inline std::string func(T &info) {                                                       \
    std::stringstream ss;                                                                  \
    ss << std::endl;                                                                       \
    for (auto &c : info.joints) {                                                          \
      ss << "  " << std::setw(25) << std::left << c.name << " : " << c.field << std::endl; \
    }                                                                                      \
    return ss.str();                                                                       \
  }

JOINT_TOOLS_GET_INFO_IMPL(GetPosition, position)
JOINT_TOOLS_GET_INFO_IMPL(GetVelocity, velocity)
JOINT_TOOLS_GET_INFO_IMPL(GetEffort, effort)

JOINT_TOOLS_GET_NAME_INFO_IMPL(GetNamePosition, position)
JOINT_TOOLS_GET_NAME_INFO_IMPL(GetNameVelocity, velocity)
JOINT_TOOLS_GET_NAME_INFO_IMPL(GetNameEffort, effort)

#define JOINT_TOOLS_PRINT_N_INFO(N, ...)                                            \
  {                                                                                 \
    static uint32_t C = 0;                                                          \
    static uint32_t num = []() {                                                    \
      auto env = std::getenv("SIM_PRINT_JOINT_INFO_N");                             \
      if (env) {                                                                    \
        return std::stoi(env);                                                      \
      }                                                                             \
      return N;                                                                     \
    }();                                                                            \
    if ((C++) < num) {                                                              \
      std::cout << "[" << C << "] " << aimrt_fmt::format(__VA_ARGS__) << std::endl; \
    }                                                                               \
  }

}  // namespace sim::joint::tools