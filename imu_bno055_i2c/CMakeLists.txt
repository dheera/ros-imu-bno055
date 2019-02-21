cmake_minimum_required(VERSION 2.8.3)
project(imu_bno055_i2c)

add_compile_options(-std=c++11 -li2c)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs std_msgs)

catkin_package(INCLUDE_DIRS include)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(imu_bno055_i2c_node src/bno055_i2c_activity.cpp src/imu_bno055_i2c_node.cpp src/watchdog.cpp)

include_directories(${catkin_INCLUDE_DIRS})

target_link_libraries(imu_bno055_i2c_node ${catkin_LIBRARIES})

install(TARGETS imu_bno055_i2c_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

