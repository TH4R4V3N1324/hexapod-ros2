cmake_minimum_required(VERSION 3.8)
project(hexapod_hardware_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(include)

# ros2_control plugin for the ST3215 bus servos
add_library(${PROJECT_NAME} SHARED
    src/st3215_bus.cpp
    src/st3215_system.cpp
)
ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    rclcpp_lifecycle
    hardware_interface
    pluginlib
)
pluginlib_export_plugin_description_file(hardware_interface hexapod_hardware_interface.xml)

# I2C sensor drivers and their nodes
add_library(i2c_sensors STATIC
    src/i2c_device.cpp
    src/ism330dhcx.cpp
    src/ina226.cpp
)
set_target_properties(i2c_sensors PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_executable(ism330dhcx_node src/ism330dhcx_node.cpp)
target_link_libraries(ism330dhcx_node i2c_sensors)
ament_target_dependencies(ism330dhcx_node rclcpp sensor_msgs)

add_executable(ina226_node src/ina226_node.cpp)
target_link_libraries(ina226_node i2c_sensors)
ament_target_dependencies(ina226_node rclcpp sensor_msgs)

# Hardware test programs, run on the robot without ROS (see hardware_tests/README.md)
add_executable(test_imu hardware_tests/test_imu.cpp)
target_link_libraries(test_imu i2c_sensors)

add_executable(test_power hardware_tests/test_power.cpp)
target_link_libraries(test_power i2c_sensors)

add_executable(test_servos hardware_tests/test_servos.cpp src/st3215_bus.cpp)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS ism330dhcx_node ina226_node test_imu test_power test_servos DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS hardware_tests/test_lidar.py DESTINATION lib/${PROJECT_NAME} RENAME test_lidar)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(hardware_interface pluginlib rclcpp rclcpp_lifecycle)

ament_package()
