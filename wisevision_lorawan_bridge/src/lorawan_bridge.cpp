/*
 * Copyright (C) 2025 wisevision
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#include "wisevision_lorawan_bridge/lorawan_bridge.hpp"

#include <grpcpp/create_channel.h>

#include "chirpstack_api/integration/integration.pb.h"
#include "wisevision_lorawan_bridge/utils.hpp"

namespace wisevision {
  LoraWanBridge::LoraWanBridge(const rclcpp::NodeOptions& options)
    : Node("lorawan_bridge",
           rclcpp::NodeOptions(options).start_parameter_services(false).start_parameter_event_publisher(false)) {
    setupParameters();
    m_mqtt_client = std::make_shared<mqtt::async_client>(m_mqtt_broker_parameters.host + ":" +
                                                             std::to_string(m_mqtt_broker_parameters.port),
                                                         "lorawan_bridge");
    m_mqtt_client->set_callback(*this);
    m_mqtt_client->connect();
    if (!connectToApi(m_api_parameters)) {
      throw std::runtime_error("Cannot connect to API. Exiting...");
    }

    m_one_off_initialization_timer_callback_group =
        this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    m_one_off_initalization_timer = this->create_wall_timer(std::chrono::nanoseconds(1),
                                                            std::bind(&LoraWanBridge::initializationCallback, this),
                                                            m_one_off_initialization_timer_callback_group);
  }

  LoraWanBridge::~LoraWanBridge() {
    RCLCPP_INFO(this->get_logger(), "LoraWanBridge shutdown.");
  }

  void LoraWanBridge::initializationCallback() {
    if (m_one_off_initalization_timer != nullptr) {
      m_one_off_initalization_timer->cancel();
      m_one_off_initialization_timer_callback_group.reset();
    }
    auto devices_opt = initializeDevices();
    if (!devices_opt.has_value()) {
      throw std::runtime_error("Cannot initialize devices. Exiting...");
    }
    m_devices = std::move(devices_opt.value());
    RCLCPP_INFO(this->get_logger(), "LoraWanBridge started successfully");
  }

  void LoraWanBridge::connected(const mqtt::string& cause) {
    RCLCPP_DEBUG(this->get_logger(), "Connected to MQTT broker (%s)", cause.c_str());
    m_mqtt_client->subscribe("application/" + m_application_id + "/#", 0);
  }

  void LoraWanBridge::message_arrived(mqtt::const_message_ptr msg) {
    RCLCPP_DEBUG(this->get_logger(), "Received MQTT message from topic: %s", msg->get_topic().c_str());
    const auto topic = msg->get_topic();
    const auto device_eui = getDeviceEuiFromTopic(topic);
    if (m_devices.find(device_eui) == m_devices.end()) {
      RCLCPP_WARN(this->get_logger(),
                  "There is no device defined with EUI \"%s\" from topic \"%s\"",
                  device_eui.c_str(),
                  topic.c_str());
      return;
    }

    const auto event_type = getEventTypeFromTopic(topic);
    switch (event_type) {
    case EventType::Up: {
      const auto raw_payload = msg->get_payload();
      integration::UplinkEvent uplink;
      if (!uplink.ParseFromString(raw_payload)) {
        RCLCPP_ERROR(this->get_logger(), "Cannot parse uplink event message.");
        return;
      }
      const auto& frm_payload = uplink.data();
      m_devices[device_eui]->publishUplink({frm_payload.begin(), frm_payload.end()});
      break;
    }
    default:
      // TODO(styczen): Implement other event types.
      RCLCPP_WARN(this->get_logger(), "Received currently unsupported event type.");
      break;
    }
  }

  bool LoraWanBridge::connectToApi(const ClientConfiguration& configuration) {
    const auto channel = grpc::CreateChannel(configuration.host + ":" + std::to_string(configuration.port),
                                             grpc::InsecureChannelCredentials());
    m_device_client = api::DeviceService::NewStub(channel);
    // TODO(styczen): Check if it is possible to do health check the server.
    return true;
  }

  std::optional<std::unordered_map<std::string, Device::UniquePtr>> LoraWanBridge::initializeDevices() {
    const auto devices_items_opt = getRegisteredDevicesItemsFromApi();
    if (!devices_items_opt.has_value()) {
      RCLCPP_FATAL(this->get_logger(), "Cannot get devices items from the API.");
      return std::nullopt;
    }
    const auto& devices_items = devices_items_opt.value();
    std::unordered_map<std::string, Device::UniquePtr> devices;
    for (const auto& device_item : devices_items) {
      const auto& dev_eui = device_item.dev_eui();
      const auto& dev_profile = device_item.device_profile_name();
      RCLCPP_DEBUG(this->get_logger(),
                   "Initializing device with EUI %s and profile name: %s",
                   dev_eui.c_str(),
                   dev_profile.c_str());
      auto dev = std::make_unique<Device>(
          this->shared_from_this(),
          dev_eui,
          std::bind(&LoraWanBridge::publishToMqtt, this, std::placeholders::_1, std::placeholders::_2));
      if (!dev->loadStandardParser()) {
        RCLCPP_WARN(this->get_logger(),
                    "Cannot load standard parser. Skipping saving device with EUI \"%s\" and profile \"%s\"",
                    dev_eui.c_str(),
                    dev_profile.c_str());
        continue;
      }
      if (!m_only_standard_parser) {
        RCLCPP_DEBUG(this->get_logger(),
                     "Attemping to load custom parser (%s) for device EUI \"%s\"",
                     dev_profile.c_str(),
                     dev_eui.c_str());
        if (!dev->loadCustomParser(dev_profile)) {
          RCLCPP_WARN(this->get_logger(),
                      "Cannot load custom parser (%s) for device EUI \"%s\". Using only standard parser.",
                      dev_profile.c_str(),
                      dev_eui.c_str());
        }
      }
      devices.insert({dev_eui, std::move(dev)});
    }
    return devices;
  }

  std::optional<std::vector<api::DeviceListItem>> LoraWanBridge::getRegisteredDevicesItemsFromApi() {
    if (m_device_client == nullptr) {
      RCLCPP_FATAL(this->get_logger(), "API client is not initialized. Cannot read devices configuration.");
      return std::nullopt;
    }
    std::vector<api::DeviceListItem> devices_items;

    constexpr size_t devices_items_limit = 10;
    api::ListDevicesRequest list_devices_req;
    list_devices_req.set_limit(devices_items_limit);
    list_devices_req.set_application_id(m_application_id);
    size_t number_of_reads = 0;
    while (true) {
      list_devices_req.set_offset(number_of_reads * devices_items_limit);
      api::ListDevicesResponse list_devices_res;
      grpc::ClientContext ctx;
      ctx.AddMetadata("authorization", "Bearer " + m_api_token);
      auto status = m_device_client->List(&ctx, list_devices_req, &list_devices_res);
      if (!status.ok()) {
        const auto& error_code = status.error_code();
        const auto& error_message = status.error_message();
        RCLCPP_ERROR(this->get_logger(),
                     "Error occured while reading from API. Code: %d. Message: %s",
                     error_code,
                     error_message.c_str());
        return std::nullopt;
      } else {
        const auto& devices = list_devices_res.result();
        if (devices.size() == 0) {
          RCLCPP_DEBUG(this->get_logger(), "No more devices items to read from the API");
          break;
        }
        devices_items.insert(devices_items.end(), devices.begin(), devices.end());
      }
      number_of_reads++;
    }
    RCLCPP_DEBUG(this->get_logger(), "Number of devices items: %ld", devices_items.size());
    return devices_items;
  }

  void LoraWanBridge::publishToMqtt(const std::string& device_eui, const std::vector<uint8_t>& payload) {
    integration::DownlinkCommand downlink;
    downlink.set_dev_eui(device_eui);
    downlink.set_confirmed(false);
    downlink.set_f_port(1);
    downlink.set_data({payload.begin(), payload.end()});
    std::string serialized_downlink;
    if (!downlink.SerializeToString(&serialized_downlink)) {
      RCLCPP_ERROR(this->get_logger(), "Cannot parse uplink event message.");
      return;
    }
    m_mqtt_client->publish("application/" + m_application_id + "/device/" + device_eui + "/command/down",
                           serialized_downlink);
  }

  std::string LoraWanBridge::getDeviceEuiFromTopic(const std::string& topic) {
    auto tokens = utils::splitStringByDelimiter(topic, "/");
    return tokens[3];
  }

  EventType LoraWanBridge::getEventTypeFromTopic(const std::string& topic) {
    auto tokens = utils::splitStringByDelimiter(topic, "/");
    return eventTypeFromString(tokens[5]);
  }

  void LoraWanBridge::setupParameters() {
    // Application ID
    rcl_interfaces::msg::ParameterDescriptor parameter_descriptor;
    parameter_descriptor.read_only = true;
    parameter_descriptor.type = rclcpp::ParameterType::PARAMETER_STRING;
    this->declare_parameter<std::string>("application_id", parameter_descriptor);
    if (!this->get_parameter<std::string>("application_id", m_application_id)) {
      throw std::runtime_error("Set \"application_id\" node parameter.");
    }
    RCLCPP_INFO(this->get_logger(), "Application ID: %s", m_application_id.c_str());

    // MQTT broker parameters
    // Host
    parameter_descriptor = rcl_interfaces::msg::ParameterDescriptor {};
    parameter_descriptor.read_only = true;
    parameter_descriptor.type = rclcpp::ParameterType::PARAMETER_STRING;
    const auto mqtt_broker_host =
        this->declare_parameter<std::string>("mqtt_broker.host", "localhost", parameter_descriptor);
    // Port
    parameter_descriptor = rcl_interfaces::msg::ParameterDescriptor {};
    parameter_descriptor.read_only = true;
    parameter_descriptor.type = rclcpp::ParameterType::PARAMETER_INTEGER;
    const auto mqtt_broker_port = this->declare_parameter<int>("mqtt_broker.port", 1883, parameter_descriptor);
    m_mqtt_broker_parameters = ClientConfiguration {};
    m_mqtt_broker_parameters.host = mqtt_broker_host;
    m_mqtt_broker_parameters.port = static_cast<size_t>(mqtt_broker_port);
    RCLCPP_INFO(this->get_logger(),
                "MQTT broker parameters: (%s, %ld)",
                m_mqtt_broker_parameters.host.c_str(),
                m_mqtt_broker_parameters.port);

    // Use only standard parser
    parameter_descriptor = rcl_interfaces::msg::ParameterDescriptor {};
    parameter_descriptor.read_only = true;
    parameter_descriptor.type = rclcpp::ParameterType::PARAMETER_BOOL;
    m_only_standard_parser = this->declare_parameter<bool>("use_only_standard", true, parameter_descriptor);
    // this->get_parameter<bool>("use_only_standard", m_only_standard_parser);
    if (m_only_standard_parser) {
      RCLCPP_INFO(this->get_logger(), "Only standard parsers will be used.");
    } else {
      RCLCPP_INFO(this->get_logger(), "Standard and custom parsers will be used.");
    }

    // API token
    const auto token = std::getenv("CHIRPSTACK_API_TOKEN");
    if (token == nullptr) {
      throw std::runtime_error("Provide API token as \"CHIRPSTACK_API_TOKEN\" environment variable.");
    }
    m_api_token = token;

    // Chirpstack API connection
    // Host
    parameter_descriptor = rcl_interfaces::msg::ParameterDescriptor {};
    parameter_descriptor.read_only = true;
    parameter_descriptor.type = rclcpp::ParameterType::PARAMETER_STRING;
    const auto api_host = this->declare_parameter<std::string>("api.host", "localhost", parameter_descriptor);
    // Port
    parameter_descriptor = rcl_interfaces::msg::ParameterDescriptor {};
    parameter_descriptor.read_only = true;
    parameter_descriptor.type = rclcpp::ParameterType::PARAMETER_INTEGER;
    const auto api_port = this->declare_parameter<int>("api.port", 8080, parameter_descriptor);
    m_api_parameters = ClientConfiguration {};
    m_api_parameters.host = api_host;
    m_api_parameters.port = static_cast<size_t>(api_port);
    RCLCPP_INFO(this->get_logger(),
                "Chirpstack API parameters: (%s, %ld)",
                m_api_parameters.host.c_str(),
                m_api_parameters.port);
  }

  EventType eventTypeFromString(const std::string& event_type) {
    if (event_type == "up") {
      return EventType::Up;
    } else if (event_type == "status") {
      return EventType::Status;
    } else if (event_type == "join") {
      return EventType::Join;
    } else if (event_type == "ack") {
      return EventType::Ack;
    } else if (event_type == "txack") {
      return EventType::Txack;
    } else if (event_type == "log") {
      return EventType::Log;
    } else if (event_type == "location") {
      return EventType::Location;
    } else if (event_type == "integration") {
      return EventType::Integration;
    }
    return EventType::Unsupported;
  }
} // namespace wisevision

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(wisevision::LoraWanBridge)
