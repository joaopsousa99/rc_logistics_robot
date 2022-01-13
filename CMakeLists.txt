cmake_minimum_required(VERSION 3.0.2)
project(rc_logistics_robot)

set (OpenCV_DIR /usr/lib/x86_64-linux-gnu/cmake/opencv4)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_ros
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)