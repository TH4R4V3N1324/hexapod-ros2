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
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)

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

add_library(height_map SHARED
    src/height_map.cpp
)
ament_target_dependencies(height_map Eigen3)

add_executable(gait_controller src/gait_controller.cpp)
target_link_libraries(gait_controller kinematic_solver trajectory_generator leg_odometry height_map)
ament_target_dependencies(gait_controller
  rclcpp
  hexapod_interfaces
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_srvs
  std_msgs
  tf2_ros
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
  height_map
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

  ament_add_gtest(test_height_map test/test_height_map.cpp)
  target_link_libraries(test_height_map height_map)
  ament_target_dependencies(test_height_map Eigen3)

  ament_add_gtest(test_kinematics test/test_kinematics.cpp)
  target_link_libraries(test_kinematics kinematic_solver)
  ament_target_dependencies(test_kinematics rclcpp hexapod_interfaces Eigen3)

  ament_add_gtest(test_foot_force test/test_foot_force.cpp)
  target_link_libraries(test_foot_force kinematic_solver)
  ament_target_dependencies(test_foot_force rclcpp hexapod_interfaces Eigen3)

  ament_add_gtest(test_body_pose test/test_body_pose.cpp)
  target_link_libraries(test_body_pose trajectory_generator)
  ament_target_dependencies(test_body_pose rclcpp hexapod_interfaces geometry_msgs Eigen3)

  ament_add_gtest(test_leg_odometry test/test_leg_odometry.cpp)
  target_link_libraries(test_leg_odometry leg_odometry)
  ament_target_dependencies(test_leg_odometry Eigen3)
endif()
install(DIRECTORY include/ DESTINATION include)

ament_package()
