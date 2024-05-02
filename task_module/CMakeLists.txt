cmake_minimum_required(VERSION 2.8.3)
project(task_module)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)
catkin_metapackage()
add_service_files(
  FILES
  PIDParams.srv
)