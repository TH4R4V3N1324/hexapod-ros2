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

add_library(kinematic_solver SHARED
    src/kinematic_solver.cpp
)

ament_target_dependencies(kinematic_solver
  rclcpp
  hexapod_interfaces
)

add_executable(gait_controller src/gait_controller.cpp)
target_link_libraries(gait_controller kinematic_solver)
ament_target_dependencies(gait_controller
  rclcpp
  hexapod_interfaces
)

install(TARGETS
  kinematic_solver
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS gait_controller DESTINATION lib/${PROJECT_NAME}/)
install(DIRECTORY include/ DESTINATION include)

ament_package()
