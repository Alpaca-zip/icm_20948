cmake_minimum_required(VERSION 3.0.2)
project(icm_20948)
set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  serial
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    sensor_msgs
    serial
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(imu_node src/imu_node.cpp)
target_link_libraries(imu_node
  ${catkin_LIBRARIES}
)
