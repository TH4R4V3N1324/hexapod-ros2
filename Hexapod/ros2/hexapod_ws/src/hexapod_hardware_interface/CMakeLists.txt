cmake_minimum_required(VERSION 3.8)
project(hexapod_hardware_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(hexapod_interfaces REQUIRED)

add_executable(sensor_manager src/sensor_manager.cpp)
ament_target_dependencies(sensor_manager rclcpp sensor_msgs hexapod_interfaces)

add_executable(servo_controller src/servo_controller.cpp)
ament_target_dependencies(servo_controller rclcpp sensor_msgs std_srvs)

add_executable(controller_bridge src/controller_bridge.cpp)
ament_target_dependencies(controller_bridge rclcpp hexapod_interfaces)

install(TARGETS sensor_manager servo_controller controller_bridge DESTINATION lib/${PROJECT_NAME}/)
ament_package()
