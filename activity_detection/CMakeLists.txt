cmake_minimum_required(VERSION 2.8.3)
project(activity_detection)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_runtime
  actionlib_msgs
  actionlib
)

add_action_files(
   DIRECTORY action
   FILES activityDetection.action
)

generate_messages(
    DEPENDENCIES
    std_msgs
    actionlib_msgs
)

catkin_package(
    CATKIN_DEPENDS actionlib_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
