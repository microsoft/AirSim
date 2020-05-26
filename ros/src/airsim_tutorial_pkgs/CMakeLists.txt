cmake_minimum_required(VERSION 3.10.0)
project(airsim_tutorial_pkgs)

# todo eigen3 in AirLib already
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  mavros_msgs
  message_generation
  nav_msgs
  # nodelet
  roscpp
  rospy
  sensor_msgs
  std_msgs
  std_srvs
  tf2
  tf2_ros
)

catkin_package(
  INCLUDE_DIRS
  CATKIN_DEPENDS message_runtime roscpp std_msgs airsim_ros_pkgs  
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${airsim_ros_pkgs_INCLUDE_DIRS}
)