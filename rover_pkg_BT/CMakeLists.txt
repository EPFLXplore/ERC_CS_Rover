cmake_minimum_required(VERSION 3.8)
project(rover_pkg_BT)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set the C++ standard globally to C++17
set(CMAKE_CXX_STANDARD 17)


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(custom_msg)
find_package(behaviortree_cpp_v3 REQUIRED)

# Include directories
include_directories(
  include
)

add_library(action_server SHARED
  src/action_server.cpp)
target_include_directories(action_server PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(action_server
  "custom_msg"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components")
rclcpp_components_register_node(action_server PLUGIN "rover_pkg_BT::FibonacciActionServer" EXECUTABLE fib_action_server)
install(TARGETS
  action_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)


add_library(action_client SHARED
  src/action_client.cpp)
target_include_directories(action_client PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(action_client
  "custom_msg"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components")
rclcpp_components_register_node(action_client PLUGIN "rover_pkg_BT::FibonacciActionClient" EXECUTABLE fib_action_client)
install(TARGETS
  action_client
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

add_library(bt_action SHARED
  src/BTAction.cpp)
target_include_directories(bt_action PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(bt_action
  "custom_msg"
  "rclcpp"
  "rclcpp_action"
  "behaviortree_cpp_v3"
  "rclcpp_components")
rclcpp_components_register_node(bt_action PLUGIN "rover_pkg_BT::SendAction" EXECUTABLE bt_action_client)
install(TARGETS 
  bt_action
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# ament package
ament_package()

