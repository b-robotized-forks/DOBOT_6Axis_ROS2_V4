cmake_minimum_required(VERSION 3.20)
project(dobot_hardware_interface)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Werror=conversion -Werror=unused-but-set-variable -Werror=return-type -Werror=shadow)
endif()

if(POLICY CMP0148)
  cmake_policy(SET CMP0148 OLD)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

add_library(
  dobot_hardware_interface
  SHARED
  src/dobot_hardware_interface.cpp
)

target_include_directories(
  dobot_hardware_interface
  PUBLIC
  include
)
target_link_libraries(dobot_hardware_interface PUBLIC
  hardware_interface::hardware_interface
  hardware_interface::mock_components
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
)

# prevent pluginlib from using boost
target_compile_definitions(dobot_hardware_interface PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(
  hardware_interface dobot_hardware_interface.xml)

install(
  TARGETS
  dobot_hardware_interface
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(
  include
)
ament_export_libraries(
  dobot_hardware_interface
)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

ament_package()
