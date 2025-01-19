#pragma once

#include <mqtt/async_client.h>
#include <mqtt/delivery_token.h>
#include <wisevision_parser/parser.hpp>
// #include <parser/parser_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <unordered_map>

#include "chirpstack_api/api/device.grpc.pb.h"
#include "wisevision_lorawan_bridge/device.hpp"

namespace wisevision {
  enum class EventType {
    Up,
    Status,
    Join,
    Ack,
    Txack,
    Log,
    Location,
    Integration,
    Unsupported,
  };

  struct ClientConfiguration {
    std::string host;
    size_t port; // cppcheck-suppress unusedStructMember
  };

  EventType eventTypeFromString(const std::string& event_type);

  class LoraWanBridge : public rclcpp::Node, public virtual mqtt::callback {
  public:
    explicit LoraWanBridge(const rclcpp::NodeOptions& options);
    virtual ~LoraWanBridge();

  private:
    std::unordered_map<std::string, Device::UniquePtr> m_devices;
    std::string m_application_id;
    std::string m_api_token;
    ClientConfiguration m_mqtt_broker_parameters;
    mqtt::async_client_ptr m_mqtt_client;
    ClientConfiguration m_api_parameters;
    std::unique_ptr<api::DeviceService::Stub> m_device_client;
    bool m_only_standard_parser;
    rclcpp::TimerBase::SharedPtr m_one_off_initalization_timer;
    rclcpp::CallbackGroup::SharedPtr m_one_off_initialization_timer_callback_group;

    void setupParameters();
    std::string getDeviceEuiFromTopic(const std::string& topic);
    EventType getEventTypeFromTopic(const std::string& topic);
    void publishToMqtt(const std::string& device_eui, const std::vector<uint8_t>& payload);
    bool connectToApi(const ClientConfiguration& configuration);
    std::optional<std::unordered_map<std::string, Device::UniquePtr>> initializeDevices();
    std::optional<std::vector<api::DeviceListItem>> getRegisteredDevicesItemsFromApi();
    void initializationCallback();

    // mqtt::callback overrides
    void message_arrived(mqtt::const_message_ptr msg) override;
    void connected(const mqtt::string& cause) override;
  };
} // namespace wisevision
