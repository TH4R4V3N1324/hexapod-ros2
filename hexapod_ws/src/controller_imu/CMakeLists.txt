cmake_minimum_required(VERSION 3.8)
project(controller_imu)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
pkg_check_modules(SDL2 REQUIRED IMPORTED_TARGET sdl2)

add_executable(controller_imu src/controller_imu.cpp)
ament_target_dependencies(controller_imu rclcpp sensor_msgs)
target_link_libraries(controller_imu PkgConfig::SDL2)

install(TARGETS controller_imu DESTINATION lib/${PROJECT_NAME})

ament_package()
