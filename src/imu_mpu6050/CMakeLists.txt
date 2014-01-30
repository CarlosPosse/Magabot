cmake_minimum_required(VERSION 2.8.3)
project(imu_mpu6050)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  rosserial_arduino
  std_msgs
  geometry_msgs
  message_generation
)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
  add_message_files(
    FILES
    Quaternion_32.msg
    Vector3_32.msg
	  Inertial.msg
  )

## Generate added messages and services with any dependencies listed here
  generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
  )

###################################
## catkin specific configuration ##
###################################

catkin_package( CATKIN_DEPENDS roscpp rospy rosserial_arduino std_msgs geometry_msgs message_runtime )

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(IMU src/IMU.cpp)
target_link_libraries(IMU ${catkin_LIBRARIES})
add_dependencies(IMU imu_mpu6050_generate_messages_cpp)

