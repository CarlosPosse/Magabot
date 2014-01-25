cmake_minimum_required(VERSION 2.8.3)
project(mag_navigation)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
	move_base
  tf
)

###################################
## catkin specific configuration ##
###################################

catkin_package( CATKIN_DEPENDS roscpp rospy move_base tf )

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

