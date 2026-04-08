#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "data/common.hpp"

namespace quadrotor::data {

struct ImageData {
  Header header;
  std::uint32_t width = 0;
  std::uint32_t height = 0;
  std::string encoding = "rgb8";
  std::uint8_t is_bigendian = 0;
  std::uint32_t step = 0;
  std::vector<std::uint8_t> data;
};

}  // namespace quadrotor::data
