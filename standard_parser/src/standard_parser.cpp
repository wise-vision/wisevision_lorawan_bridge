#include "standard_parser/standard_parser.hpp"

#include "standard_parser/utils.hpp"

namespace wisevision {
  std::optional<std::vector<uint8_t>> StandardParser::rosMessageToBytes(const rclcpp::SerializedMessage& message) {
    std_msgs::msg::String ros_message;
    m_serde.deserialize_message(&message, &ros_message);
    const auto& hex_encoded_downlink = ros_message.data;
    const auto binary = utils::convertHexStringToBinary(hex_encoded_downlink);
    return binary;
  };

  std::optional<rclcpp::SerializedMessage> StandardParser::bytesToRosMessage(const std::vector<uint8_t>& bytes) {
    const auto hex_str = utils::convertBinaryToHexString(bytes);
    std_msgs::msg::String ros_message;
    ros_message.data = hex_str;
    rclcpp::SerializedMessage message;
    m_serde.serialize_message(&ros_message, &message);
    return message;
  };

  std::string StandardParser::getPublisherMessageType() const {
    return "std_msgs/msg/String";
  }

  std::string StandardParser::getSubscriptionMessageType() const {
    return "std_msgs/msg/String";
  }

  std::string StandardParser::getName() const {
    return "Standard";
  };
} // namespace wisevision

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(wisevision::StandardParser, wisevision::Parser)
