cmake_minimum_required(VERSION 2.8.3)
project(my_ros_service)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_runtime
  nav_msgs
  rospy
  std_msgs
  example_ros_service
  message_generation
)

catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES my_ros_service
#  CATKIN_DEPENDS geometry_msgs message_runtime nav_msgs rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
