#pragma once

#include <string>
#include <vector>

namespace wisevision::utils {
  std::string convertBinaryToHexString(const std::vector<uint8_t>& bytes);
  std::vector<uint8_t> convertHexStringToBinary(const std::string& str);
} // namespace wisevision::utils
