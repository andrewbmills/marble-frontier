cmake_minimum_required(VERSION 2.8.3)
project(home_planner)

find_package(catkin REQUIRED COMPONENTS
  rospy
  cartographer_ros
)
catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)