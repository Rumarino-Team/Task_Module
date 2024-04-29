cmake_minimum_required(VERSION 2.8.3)
project(tasks)

# Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
  tf2_ros
  tf2_geometry_msgs
  nav_msgs
  sensor_msgs
  stereo_msgs
  dynamic_reconfigure
  nodelet
  diagnostic_updater
  geometry_msgs
  visualization_msgs
  zed_interfaces
  message_runtime
  rospy
  smach
  smach_ros
  actionlib
  actionlib_msgs
  message_generation
  rosbag
  std_msgs
  topic_tools
  uuv_control_msgs
  nav_sensors

)

# Declare a catkin package
catkin_package(
  CATKIN_DEPENDS
  tf2_ros
  tf2_geometry_msgs
  nav_msgs
  sensor_msgs
  stereo_msgs
  dynamic_reconfigure
  nodelet
  diagnostic_updater
  geometry_msgs
  visualization_msgs
  zed_interfaces
  message_runtime
  rospy
  smach
  smach_ros
  actionlib
  actionlib_msgs
  message_generation
  rosbag
  std_msgs
  topic_tools
  uuv_control_msgs
)

# Include directories
include_directories(include ${catkin_INCLUDE_DIRS})

# Install Python scripts and nodes
install(DIRECTORY nodes scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
  FILES_MATCHING PATTERN "*.py"
  PATTERN ".svn" EXCLUDE
)

# Install launch files
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Install config files
install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
