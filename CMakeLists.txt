cmake_minimum_required(VERSION 3.16)
project(imu_bno055)

find_package(ros_environment REQUIRED)

set(ROS_VERSION $ENV{ROS_VERSION})

if(${ROS_VERSION} EQUAL 1)
  include(CMakeLists.ros1.txt)
elseif(${ROS_VERSION} EQUAL 2)
  include(CMakeLists.ros2.txt)
endif()
