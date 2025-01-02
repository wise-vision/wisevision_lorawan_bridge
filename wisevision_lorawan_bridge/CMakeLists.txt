cmake_minimum_required(VERSION 3.8)
project(wisevision_lorawan_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
# find_package(Boost REQUIRED)
find_library(PahoMqttCpp_LIBRARY libpaho-mqttpp3.so.1 REQUIRED)
find_package(chirpstack_api REQUIRED)
find_package(wisevision_plugin_loader REQUIRED)
find_package(wisevision_parser REQUIRED)

set(component_name ${PROJECT_NAME}_component)

add_library(${component_name} SHARED
  src/utils.cpp
  src/device.cpp
  src/lorawan_bridge.cpp
)
target_include_directories(${component_name} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${component_name}
  ${PahoMqttCpp_LIBRARY}
)
set(dependencies
  rclcpp
  rclcpp_components
  std_msgs
  chirpstack_api
  wisevision_plugin_loader
  wisevision_parser
)
ament_target_dependencies(${component_name}
  ${dependencies}
)
rclcpp_components_register_node(
  ${component_name}
  PLUGIN "wisevision::LoraWanBridge"
  EXECUTABLE lorawan_bridge
)
install(
  TARGETS ${component_name}
  EXPORT export_${component_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
ament_export_targets(export_${component_name} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  ${dependencies}
)

ament_package()
