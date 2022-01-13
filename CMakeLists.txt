cmake_minimum_required(VERSION 3.0.2)
project(rc_logistics_robot)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_ros
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)