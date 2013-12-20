cmake_minimum_required(VERSION 2.8.3)
project(com)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
 CATKIN_DEPENDS roscpp rospy std_msgs
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(talker src/talker.cpp)
add_executable(listener src/listener.cpp)

add_dependencies(talker com_generate_msgs_cpp)
add_dependencies(listener com_generate_msgs_cpp)

target_link_libraries(talker
  ${catkin_LIBRARIES}
)

target_link_libraries(listener
  ${catkin_LIBRARIES}
)
