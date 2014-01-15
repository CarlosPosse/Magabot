cmake_minimum_required(VERSION 2.8.3)
project(mag_window)

find_package(catkin REQUIRED COMPONENTS
  joy
  magabot
  roscpp
  rospy
  rosserial_arduino
  tf
  urg_node
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES magabot_nav
#  CATKIN_DEPENDS imu-drivers imu-tools joystick_drivers magabot move_base roscpp rospy rosserial tf urg_node xboxjoy
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

