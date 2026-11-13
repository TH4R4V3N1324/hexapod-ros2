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

# The gait maths (IK/FK, frames, trajectories, leg odometry) lives in gait_core, the
# hexapod-gait-core submodule next to this package
find_package(gait_core REQUIRED)

include_directories(include ${EIGEN3_INCLUDE_DIR})

add_library(height_map SHARED
    src/height_map.cpp
)
ament_target_dependencies(height_map Eigen3)

add_executable(gait_controller src/gait_controller.cpp)
target_link_libraries(gait_controller gait_core::gait_core height_map)
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

install(TARGETS
  height_map
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS gait_controller DESTINATION lib/${PROJECT_NAME}/)

# The gait maths is tested in hexapod-gait-core; only the ROS-side code is tested here
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_height_map test/test_height_map.cpp)
  target_link_libraries(test_height_map height_map)
  ament_target_dependencies(test_height_map Eigen3)
endif()
install(DIRECTORY include/ DESTINATION include)

ament_package()
