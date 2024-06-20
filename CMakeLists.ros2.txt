cmake_minimum_required(VERSION 3.5)
project(imu_bno055)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(std_srvs REQUIRED)

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${diagnostic_msgs_INCLUDE_DIRS}
)

add_executable(bno055_i2c_node
  src/bno055_i2c_driver.cpp
  src/bno055_i2c_node_ros2.cpp
  src/watchdog.cpp
)

ament_target_dependencies(bno055_i2c_node
  rclcpp
  sensor_msgs
  std_msgs
  diagnostic_msgs
  std_srvs
)

install(TARGETS bno055_i2c_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

