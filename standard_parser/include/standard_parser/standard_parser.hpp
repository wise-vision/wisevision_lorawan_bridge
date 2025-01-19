/*
 * Copyright (C) 2025 wisevision
 *
 * SPDX-License-Identifier: MPL-2.0
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#pragma once

#include <std_msgs/msg/string.hpp>
#include <wisevision_parser/parser.hpp>

namespace wisevision {
  class StandardParser : public Parser {
  public:
    std::optional<std::vector<uint8_t>> rosMessageToBytes(const rclcpp::SerializedMessage& message) override;
    std::optional<rclcpp::SerializedMessage> bytesToRosMessage(const std::vector<uint8_t>& bytes) override;
    std::string getPublisherMessageType() const override;
    std::string getSubscriptionMessageType() const override;
    std::string getName() const override;

  private:
    rclcpp::Serialization<std_msgs::msg::String> m_serde;
  };
} // namespace wisevision
