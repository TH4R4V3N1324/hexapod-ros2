cmake_minimum_required(VERSION 3.8)
project(hexapod_gait_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hexapod_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(include ${EIGEN3_INCLUDE_DIR})

add_library(kinematic_solver SHARED
    src/kinematic_solver.cpp
)

ament_target_dependencies(kinematic_solver
  rclcpp
  hexapod_interfaces
  Eigen3
)

add_library(trajectory_generator SHARED
    src/trajectory_generator.cpp
    src/gait_config.cpp
)

ament_target_dependencies(trajectory_generator
  rclcpp
  hexapod_interfaces
  Eigen3
  geometry_msgs
)

add_executable(gait_controller src/gait_controller.cpp)
target_link_libraries(gait_controller kinematic_solver trajectory_generator)
ament_target_dependencies(gait_controller
  rclcpp
  hexapod_interfaces
  geometry_msgs
)

install(TARGETS
  kinematic_solver
  trajectory_generator
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS gait_controller DESTINATION lib/${PROJECT_NAME}/)
install(DIRECTORY include/ DESTINATION include)

ament_package()
