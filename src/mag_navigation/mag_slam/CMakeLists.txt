cmake_minimum_required(VERSION 2.8.3)
project(mag_slam)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  rosserial_arduino
	geometry_msgs
  tf
)

###################################
## catkin specific configuration ##
###################################

catkin_package( CATKIN_DEPENDS magabot move_base roscpp rospy rosserial_arduino geometry_msgs tf )

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(Footprint src/Footprint.cpp)

add_dependencies(Footprint ${catkin_EXPORTED_TARGETS})

target_link_libraries(Footprint ${catkin_LIBRARIES})
