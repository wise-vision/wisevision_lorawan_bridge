#include "wisevision_lorawan_bridge/device.hpp"

#include <wisevision_plugin_loader/plugin_loader.hpp>

namespace wisevision {
  Device::Device(const rclcpp::Node::SharedPtr& node, const std::string& eui, MqttPublishFunction publishToMqtt)
    : m_eui(eui),
      m_node(node),
      m_publish_to_mqtt(publishToMqtt) {
  }

  bool Device::loadStandardParser() {
    m_standard_parser = plugin_loader::loadPlugin<wisevision::Parser>("standard_parser", "wisevision::Parser", "standard_parser");
    if (m_standard_parser == nullptr) {
      return false;
    }

    m_uplink_raw_pub = m_node->create_generic_publisher("eui_" + m_eui + "/uplink/raw",
                                                        m_standard_parser->getPublisherMessageType(),
                                                        rclcpp::QoS(10));
    m_downlink_raw_sub =
        m_node->create_generic_subscription("eui_" + m_eui + "/downlink/raw",
                                            m_standard_parser->getSubscriptionMessageType(),
                                            rclcpp::QoS(10),
                                            std::bind(&Device::downlinkRawCallback, this, std::placeholders::_1));
    return true;
  }

  bool Device::loadCustomParser(const std::string& parser_name) {
    m_custom_parser = plugin_loader::loadPlugin<wisevision::Parser>(parser_name, "wisevision::Parser", parser_name);
    if (m_custom_parser == nullptr) {
      return false;
    }
    m_uplink_pub = m_node->create_generic_publisher("eui_" + m_eui + "/uplink/custom",
                                                    m_custom_parser->getPublisherMessageType(),
                                                    rclcpp::QoS(10));
    m_downlink_sub =
        m_node->create_generic_subscription("eui_" + m_eui + "/downlink/custom",
                                            m_custom_parser->getSubscriptionMessageType(),
                                            rclcpp::QoS(10),
                                            std::bind(&Device::downlinkCallback, this, std::placeholders::_1));
    return true;
  }

  void Device::publishUplink(const std::vector<uint8_t>& payload) {
    auto standard_ros_frm_payload = m_standard_parser->bytesToRosMessage(payload);
    if (standard_ros_frm_payload.has_value()) {
      m_uplink_raw_pub->publish(standard_ros_frm_payload.value());
    } else {
      RCLCPP_ERROR(m_node->get_logger(), "Cannot convert raw bytes into standard ROS 2 message.");
    }

    if (m_custom_parser && m_uplink_pub) {
      RCLCPP_DEBUG(m_node->get_logger(),
                   "Parsing payload with custom parser of name \"%s\"",
                   m_custom_parser->getName().c_str());
      auto custom_ros_frm_payload = m_custom_parser->bytesToRosMessage(payload);
      if (custom_ros_frm_payload.has_value()) {
        m_uplink_pub->publish(custom_ros_frm_payload.value());
      } else {
        RCLCPP_ERROR(m_node->get_logger(), "Cannot convert raw bytes into custom ROS 2 message.");
      }
    }
  }

  void Device::downlinkRawCallback(const std::shared_ptr<rclcpp::SerializedMessage> msg) {
    const auto binary_opt = m_standard_parser->rosMessageToBytes(*msg);
    if (!binary_opt.has_value()) {
      RCLCPP_WARN(m_node->get_logger(),
                  "Cannot parse standard ROS 2 message to binary data. Skipping publishing to MQTT.");
      return;
    }
    const auto& binary = binary_opt.value();
    m_publish_to_mqtt(m_eui, binary);
  }

  void Device::downlinkCallback(const std::shared_ptr<rclcpp::SerializedMessage> msg) {
    const auto binary_opt = m_custom_parser->rosMessageToBytes(*msg);
    if (!binary_opt.has_value()) {
      RCLCPP_WARN(m_node->get_logger(),
                  "Cannot parse custom ROS 2 message to binary data. Skipping publishing to MQTT.");
      return;
    }
    const auto& binary = binary_opt.value();
    m_publish_to_mqtt(m_eui, binary);
  }
} // namespace wisevision
