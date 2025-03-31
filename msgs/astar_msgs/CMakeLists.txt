cmake_minimum_required(VERSION 2.8.3)
project(astar_msgs)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
  std_msgs
  geometry_msgs
  message_generation
  roscpp
)

## Declare ROS messages and services
add_message_files(
  FILES
  AStarPath.msg
  AStarPathArray.msg
)

## Generate added messages and services
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

## Declare a catkin package
catkin_package(
  CATKIN_DEPENDS 
  message_runtime 
  std_msgs 
  geometry_msgs
)

## Build
include_directories(
  ${catkin_INCLUDE_DIRS}
)