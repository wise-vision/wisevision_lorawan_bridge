/*
 * Copyright (C) 2025 wisevision
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#pragma once

#include <string>
#include <vector>

namespace wisevision::utils {
  std::string convertBinaryToHexString(const std::vector<uint8_t>& bytes);
  std::vector<uint8_t> convertHexStringToBinary(const std::string& str);
} // namespace wisevision::utils
