cmake_minimum_required(VERSION 2.8.3)
project(mag_joy)

find_package(catkin REQUIRED COMPONENTS
  joy
  roscpp
  std_msgs
  geometry_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS roscpp rospy joy std_msgs geometry_msgs
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(mag_joy_node src/mag_joy_node.cpp)

target_link_libraries(mag_joy_node ${catkin_LIBRARIES} )
