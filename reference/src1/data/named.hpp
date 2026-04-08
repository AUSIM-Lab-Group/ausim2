#pragma once

#include <string>

namespace sim::data {

template <class T>
struct Named : public T {
  std::string name;
};

}  // namespace sim::data