cmake_minimum_required(VERSION 2.8.3)
project(robotics_controller)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  dynamixel_controllers
  dynamixel_msgs
  roscpp
  rospy
  std_msgs
  trajectory_msgs
  control_msgs
  message_generation
)

add_message_files(
  DIRECTORY msg
  FILES
  Potentiometer.msg
  Trajectory.msg
)

add_service_files(
   DIRECTORY srv
   FILES
   move_arm.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES robotics_controller
   CATKIN_DEPENDS dynamixel_controllers roscpp rospy std_msgs message_runtime
#  DEPENDS system_lib
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

