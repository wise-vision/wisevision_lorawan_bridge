cmake_minimum_required(VERSION 3.8)
project(chirpstack_api)

if(NOT DEFINED ENV{GRPC_INSTALL_DIR})
  message(FATAL_ERROR "GRPC_INSTALL_DIR environment variable must be set.")
endif()

find_package(ament_cmake REQUIRED)
find_package(Protobuf CONFIG REQUIRED)
find_package(gRPC CONFIG REQUIRED)

set(generated_proto_source ${PROJECT_BINARY_DIR}/generated)

execute_process(
  COMMAND bash -c "cd ${PROJECT_SOURCE_DIR} && ./setup_proto.sh"
)

execute_process(
    COMMAND bash -c "mkdir -p ${generated_proto_source} && $ENV{GRPC_INSTALL_DIR}/bin/protoc -I=${PROJECT_SOURCE_DIR}/proto --cpp_out=${generated_proto_source} --grpc_out=${generated_proto_source} --plugin=protoc-gen-grpc=$ENV{GRPC_INSTALL_DIR}/bin/grpc_cpp_plugin $(find ${PROJECT_SOURCE_DIR}/proto -iname *.proto)"
)

file(GLOB_RECURSE ALL_PROTO_SRCS "${generated_proto_source}/*.pb.cc")
file(GLOB_RECURSE ALL_PROTO_HDRS "${generated_proto_source}/*.pb.h")

set(ament_dependencies
  Protobuf
)

add_library(${PROJECT_NAME} SHARED
  ${ALL_PROTO_HDRS}
  ${ALL_PROTO_SRCS}
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${generated_proto_source}>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME}
  ${ament_dependencies}
)
target_link_libraries(${PROJECT_NAME}
  gRPC::grpc++
)
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

foreach(HDR ${ALL_PROTO_HDRS})
  file(RELATIVE_PATH REL_FIL ${generated_proto_source} ${HDR})
  get_filename_component(DIR ${REL_FIL} DIRECTORY)
  install(
    FILES ${HDR}
    DESTINATION include/${DIR}
  )
endforeach()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${ament_dependencies})

ament_package(
  CONFIG_EXTRAS
  chirpstack_api-extras.cmake
)
