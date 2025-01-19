#include "wisevision_lorawan_bridge/utils.hpp"

namespace wisevision::utils {
  std::vector<std::string> splitStringByDelimiter(const std::string& str, const std::string& delimiter) {
    std::vector<std::string> tokens;
    size_t last = 0;
    size_t next = 0;
    while ((next = str.find(delimiter, last)) != std::string::npos) {
      tokens.push_back(str.substr(last, next - last));
      last = next + 1;
    }
    tokens.push_back(str.substr(last));
    return tokens;
  }
} // namespace wisevision::utils
