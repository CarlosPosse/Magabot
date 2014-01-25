cmake_minimum_required(VERSION 2.8.3)
project(mag_simulation)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
)

###################################
## catkin specific configuration ##
###################################

catkin_package( CATKIN_DEPENDS roscpp rospy )

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

