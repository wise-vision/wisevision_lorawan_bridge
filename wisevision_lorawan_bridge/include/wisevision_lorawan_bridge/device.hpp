#pragma once

#include <functional>
#include <wisevision_parser/parser.hpp>
#include <rclcpp/generic_publisher.hpp>
#include <rclcpp/generic_subscription.hpp>
#include <rclcpp/node.hpp>

namespace wisevision {
  using MqttPublishFunction = std::function<void(const std::string&, const std::vector<uint8_t>&)>;
  class Device {
  public:
    using SharedPtr = std::shared_ptr<Device>;
    using UniquePtr = std::unique_ptr<Device>;

    explicit Device(const rclcpp::Node::SharedPtr& node, const std::string& eui, MqttPublishFunction publishToMqtt);
    ~Device() = default;
    Device(Device& other) = delete;
    Device& operator=(Device& other) = delete;
    Device operator=(Device other) = delete;
    void publishUplink(const std::vector<uint8_t>& payload);
    bool loadStandardParser();
    bool loadCustomParser(const std::string& parser_name);

  private:
    std::string m_eui;

    rclcpp::Node::SharedPtr m_node;

    Parser::UniquePtr m_standard_parser;
    Parser::UniquePtr m_custom_parser;

    rclcpp::GenericPublisher::SharedPtr m_uplink_raw_pub;
    rclcpp::GenericSubscription::SharedPtr m_downlink_raw_sub;

    rclcpp::GenericPublisher::SharedPtr m_uplink_pub;
    rclcpp::GenericSubscription::SharedPtr m_downlink_sub;

    MqttPublishFunction m_publish_to_mqtt;

    void downlinkRawCallback(const std::shared_ptr<rclcpp::SerializedMessage> msg);
    void downlinkCallback(const std::shared_ptr<rclcpp::SerializedMessage> msg);
  };
} // namespace wisevision
