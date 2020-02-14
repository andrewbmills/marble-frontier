cmake_minimum_required(VERSION 2.8.3)
project(home_planner)

find_package(catkin REQUIRED COMPONENTS
  rospy
  cartographer_ros
  cartographer_ros_msgs
)
find_package(cartographer)

catkin_package(
  CATKIN_DEPENDS
  rospy
  cartographer_ros
  cartographer_ros_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
