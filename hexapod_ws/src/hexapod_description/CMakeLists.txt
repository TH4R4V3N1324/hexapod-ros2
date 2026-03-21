cmake_minimum_required(VERSION 3.8)
project(hexapod_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(joint_state_relay src/joint_state_relay.cpp)
ament_target_dependencies(joint_state_relay
	rclcpp 
	sensor_msgs 
	std_msgs
)

install (TARGETS joint_state_relay
	DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY 
	config
  urdf 
  launch
  meshes 
  rviz
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
