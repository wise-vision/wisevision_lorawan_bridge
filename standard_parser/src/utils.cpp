/*
 * Copyright (C) 2025 wisevision
 *
 * SPDX-License-Identifier: MPL-2.0
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#include "standard_parser/utils.hpp"

#include <algorithm>
#include <cctype>
#include <iomanip>
#include <sstream>
#include <string>

namespace wisevision::utils {
  std::string convertBinaryToHexString(const std::vector<uint8_t>& bytes) {
    std::ostringstream hex_stream;
    hex_stream << std::hex << std::setfill('0');
    for (const auto c : bytes) {
      hex_stream << std::setw(2) << static_cast<int>(c);
    }
    return hex_stream.str();
  }

  std::optional<std::vector<uint8_t>> convertHexStringToBinary(const std::string& str) {
    if (str.size() % 2 != 0) {
      return std::nullopt;
    }
    if (!std::all_of(str.begin(), str.end(), [](const unsigned char c) { return std::isxdigit(c) != 0; })) {
      return std::nullopt;
    }

    std::vector<uint8_t> binary(str.size() / 2);
    for (size_t i = 0; i < str.size(); i += 2) {
      binary[i / 2] = static_cast<uint8_t>(std::stoi(str.substr(i, 2), nullptr, 16));
    }
    return binary;
  }
} // namespace wisevision::utils
