cmake_minimum_required(VERSION 2.8.3)
project(home_planner)

find_package(catkin REQUIRED COMPONENTS
  rospy
  message_generation
  std_msgs
  geometry_msgs
  nav_msgs
  lcd_pkg
  tf
)

add_message_files(
   FILES
   PoseGraph.msg
)

generate_messages(
   DEPENDENCIES
   geometry_msgs
   roscpp
)

catkin_package(
  CATKIN_DEPENDS
  rospy
  message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
