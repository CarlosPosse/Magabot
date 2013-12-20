cmake_minimum_required(VERSION 2.8.3)
project(xboxjoy)

find_package(catkin REQUIRED COMPONENTS
  joy
  roscpp
  std_msgs
  message_generation
  geometry_msgs
)

#######################################
## Declare ROS messages and services ##
#######################################

add_message_files(
  FILES
  Control.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
  geometry_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS message_runtime roscpp rospy joy std_msgs geometry_msgs
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(xboxjoy_node src/xboxjoy_node.cpp)

target_link_libraries(xboxjoy_node
  ${catkin_LIBRARIES}
)
