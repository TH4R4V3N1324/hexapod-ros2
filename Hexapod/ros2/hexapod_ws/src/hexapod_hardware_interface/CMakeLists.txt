cmake_minimum_required(VERSION 3.8)
project(hexapod_hardware_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(hexapod_interfaces REQUIRED)

include_directories(include)

add_library(hardware_handlers SHARED
    src/sensor_handler.cpp
    src/servo_handler.cpp
    src/controller_handler.cpp
)

ament_target_dependencies(hardware_handlers
    rclcpp
    sensor_msgs
    geometry_msgs
    hexapod_interfaces
)

add_executable(esp_bridge_fake src/esp_bridge_fake.cpp)
target_link_libraries(esp_bridge_fake hardware_handlers)
ament_target_dependencies(esp_bridge_fake
    rclcpp
    sensor_msgs
    geometry_msgs
    hexapod_interfaces
)

install(TARGETS
  hardware_handlers
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION bin/${PROJECT_NAME}
)

install(TARGETS esp_bridge_fake DESTINATION lib/${PROJECT_NAME}/)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
