cmake_minimum_required(VERSION 3.5)
project(cr_robot_ros2)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_DEPS
    rclcpp
    rclcpp_action
    rclcpp_lifecycle
    std_msgs
    sensor_msgs
    dobot_msgs_v4
    hardware_interface
    pluginlib
)

# find dependencies
find_package(ament_cmake REQUIRED)

foreach(PACKAGE ${THIS_PACKAGE_DEPS})
  find_package(${PACKAGE} REQUIRED)
endforeach()


# ---------------------------------------------------------
# 1. Shared Dobot Driver Library
#    Contains the TCP comms and parsing logic (CRCommanderRos2)
# ---------------------------------------------------------
add_library(dobot_driver SHARED 
  src/command.cpp 
  src/tcp_socket.cpp 
  src/parseTool.cpp
)

target_include_directories(dobot_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(dobot_driver
  ${THIS_PACKAGE_DEPS}
)

target_link_libraries(dobot_driver pthread)

# ---------------------------------------------------------
# 2. ROS2 Control Hardware Interface Plugin
# ---------------------------------------------------------
add_library(dobot_hardware_interface SHARED
  src/dobot_hardware_interface.cpp
  src/dobot_hardware_interface_gpio.cpp
)

target_include_directories(dobot_hardware_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(dobot_hardware_interface
  dobot_driver
)

ament_target_dependencies(dobot_hardware_interface
  ${THIS_PACKAGE_DEPS}
)

# Prevent pluginlib from using boost
target_compile_definitions(dobot_hardware_interface PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

# ---------------------------------------------------------
# 3. ROS2 Dashboard Node
# ---------------------------------------------------------
add_executable(dobot_dashboard_node src/main.cpp src/cr_robot_ros2.cpp)

target_include_directories(dobot_dashboard_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>) 

ament_target_dependencies(
  dobot_dashboard_node
  ${THIS_PACKAGE_DEPS}
)

target_link_libraries(dobot_dashboard_node 
        dobot_driver
        pthread
        )

# ---------------------------------------------------------
# Install & Export
# ---------------------------------------------------------

install(TARGETS 
  dobot_driver 
  dobot_hardware_interface
  dobot_dashboard_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(
  hardware_interface dobot_hardware_interface.xml)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(dobot_driver dobot_hardware_interface)
ament_export_dependencies(
  hardware_interface 
  pluginlib 
  rclcpp 
  rclcpp_lifecycle 
  dobot_msgs_v4
)

ament_package()
