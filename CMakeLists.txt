cmake_minimum_required(VERSION 2.8.3)
project(imu_bno055)

add_compile_options(-std=c++11 -li2c)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs std_msgs diagnostic_msgs)

catkin_package(INCLUDE_DIRS include)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(bno055_i2c_node src/bno055_i2c_activity.cpp src/bno055_i2c_node.cpp src/watchdog.cpp)
#add_dependencies(bno055_i2c_node diagnostic_msgs_gencpp)

include_directories(${catkin_INCLUDE_DIRS})

target_link_libraries(bno055_i2c_node ${catkin_LIBRARIES})

install(TARGETS bno055_i2c_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

