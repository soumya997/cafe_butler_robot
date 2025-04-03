cmake_minimum_required(VERSION 3.0.2)
project(cafe_butler_robot)

find_package(catkin REQUIRED COMPONENTS
  rospy
  actionlib
  actionlib_msgs
  std_msgs
  geometry_msgs
  message_generation
)

add_action_files(
  FILES
  DeliveryTask.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  rospy
  actionlib
  actionlib_msgs
  std_msgs
  geometry_msgs
  message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/butler_robot_controller.py
  scripts/test_client.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
) 