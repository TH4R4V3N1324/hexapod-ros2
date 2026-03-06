cmake_minimum_required(VERSION 3.8)
project(hexapod_teleop)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(hexapod_interfaces REQUIRED)

add_executable(keyboard_teleop src/keyboard_teleop.cpp)
ament_target_dependencies(keyboard_teleop
  rclcpp
  geometry_msgs
  hexapod_interfaces
)

add_executable(controller_teleop src/controller_teleop.cpp)
ament_target_dependencies(controller_teleop
  rclcpp
  geometry_msgs
  sensor_msgs
  hexapod_interfaces
)

install(TARGETS
  keyboard_teleop
  controller_teleop
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(TARGETS keyboard_teleop controller_teleop DESTINATION lib/${PROJECT_NAME}/)
ament_package()
