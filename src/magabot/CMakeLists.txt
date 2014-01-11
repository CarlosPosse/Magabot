cmake_minimum_required(VERSION 2.8.3)
project(magabot)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  rosserial_arduino
  message_generation
  std_msgs
  nav_msgs
  geometry_msgs
  tf
)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  motorClick.msg
  motorSpeed.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  nav_msgs
  geometry_msgs
  tf
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS roscpp rospy rosserial_arduino message_runtime std_msgs nav_msgs geometry_msgs tf
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(Control src/Control.cpp)
add_executable(Odometry src/Odometry.cpp)

add_dependencies(Control ${catkin_EXPORTED_TARGETS})
add_dependencies(Odometry ${catkin_EXPORTED_TARGETS})

target_link_libraries(Control ${catkin_LIBRARIES})
target_link_libraries(Odometry ${catkin_LIBRARIES})

