################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.0.2)
project(icm_20948)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
)

################################################################################
# Declare catkin specific configuration to be passed to dependent projects
################################################################################
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs
)

################################################################################
# Build
################################################################################
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(icm_20948_example src/icm_20948_example.cpp)
target_link_libraries(icm_20948_example ${catkin_LIBRARIES})