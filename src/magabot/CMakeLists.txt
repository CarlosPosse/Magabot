cmake_minimum_required(VERSION 2.8.3)
project(magabot)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  rosserial_arduino
  std_msgs
  message_generation
)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Encoders.msg
  Velocity.msg
  Color.msg
  Status.msg
  Actuation.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package( LIBRARIES magabot
   CATKIN_DEPENDS roscpp rospy rosserial_arduino std_msgs message_runtime
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(Odometry src/Odometry.cpp)
target_link_libraries(Odometry ${catkin_LIBRARIES})
add_dependencies(Odometry ${catkin_EXPORTED_TARGETS})

add_executable(Controller src/Controller.cpp)
target_link_libraries(Controller ${catkin_LIBRARIES})
add_dependencies(Controller ${catkin_EXPORTED_TARGETS})

