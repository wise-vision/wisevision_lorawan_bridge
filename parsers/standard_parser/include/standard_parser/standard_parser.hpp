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
