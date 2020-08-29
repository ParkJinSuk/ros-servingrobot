cmake_minimum_required(VERSION 3.0.2)
project(ros-servingrobot)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
)

catkin_package(
  LIBRARIES ros-servingrobot
)

add_executable(imu_wt61c_publisher node/imu_wt61c_publisher.cpp)
target_link_libraries(imu_wt61c_publisher ${catkin_LIBRARIES})


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

