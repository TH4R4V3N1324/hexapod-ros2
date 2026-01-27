cmake_minimum_required(VERSION 3.8)
project(hexapod_gait_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hexapod_interfaces REQUIRED)

include_directories(include)

add_library(kinematic_solver_service SHARED
    src/kinematic_solver.cpp
)

ament_target_dependencies(kinematic_solver_service
  rclcpp
  hexapod_interfaces
)


ament_package()
