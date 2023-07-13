cmake_minimum_required(VERSION 3.5)
project(icm_20948)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(serial REQUIRED)

include_directories(include)

add_executable(imu_node src/imu_node.cpp)
ament_target_dependencies(imu_node rclcpp sensor_msgs serial)

install(TARGETS
  imu_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  rviz2
  DESTINATION share/${PROJECT_NAME})

ament_package()
