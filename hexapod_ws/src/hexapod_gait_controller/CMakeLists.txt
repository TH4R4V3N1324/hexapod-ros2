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
find_package(nav_msgs REQUIRED)

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

add_library(leg_odometry SHARED
    src/leg_odometry.cpp
)
ament_target_dependencies(leg_odometry Eigen3)

add_executable(gait_controller src/gait_controller.cpp)
target_link_libraries(gait_controller kinematic_solver trajectory_generator leg_odometry)
ament_target_dependencies(gait_controller
  rclcpp
  hexapod_interfaces
  geometry_msgs
  nav_msgs
)

add_executable(leg_axis_tester src/leg_axis_test.cpp)
target_link_libraries(leg_axis_tester kinematic_solver trajectory_generator)
ament_target_dependencies(leg_axis_tester
  rclcpp
  hexapod_interfaces
  geometry_msgs
)

install(TARGETS
  kinematic_solver
  trajectory_generator
  leg_odometry
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS gait_controller leg_axis_tester DESTINATION lib/${PROJECT_NAME}/)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_foot_directions test/test_foot_directions.cpp)
  target_link_libraries(test_foot_directions trajectory_generator)
  ament_target_dependencies(test_foot_directions rclcpp hexapod_interfaces geometry_msgs Eigen3)

  ament_add_gtest(test_leg_clearance test/test_leg_clearance.cpp)
  target_link_libraries(test_leg_clearance trajectory_generator kinematic_solver)
  ament_target_dependencies(test_leg_clearance rclcpp hexapod_interfaces geometry_msgs Eigen3)

  ament_add_gtest(test_leg_odometry test/test_leg_odometry.cpp)
  target_link_libraries(test_leg_odometry leg_odometry)
  ament_target_dependencies(test_leg_odometry Eigen3)
endif()
install(DIRECTORY include/ DESTINATION include)

ament_package()
