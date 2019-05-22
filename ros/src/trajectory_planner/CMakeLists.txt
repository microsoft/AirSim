cmake_minimum_required(VERSION 3.10.0)
project(trajectory_planner)

find_package(catkin REQUIRED COMPONENTS
  # airsim_ros_msgs
  airsim_ros_pkgs
  mav_msgs
  mav_trajectory_generation
  roscpp
  rospy
  tf2
  tf2_ros
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime roscpp std_msgs# nodelet 
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${mav_trajectory_generation_INCLUDE_DIRS}
  ${mav_msgs_INCLUDE_DIRS}
  ${airsim_ros_pkgs_INCLUDE_DIRS}
)

add_library(traj_gen_helper src/traj_gen_helper.cpp)
add_dependencies(traj_gen_helper ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(traj_gen_helper ${catkin_EXPORTED_TARGETS})

# add_executable(pd_position_controller_simple_node src/pd_position_controller_simple_node.cpp)
# target_link_libraries(pd_position_controller_simple_node pd_position_controller_simple airsim_ros ${catkin_LIBRARIES} AirLib)

install(TARGETS
    # list of nodes
    traj_gen_helper
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# for shipping binaries. run `$ catin_make install`
install(DIRECTORY
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  # PATTERN ".git" EXCLUDE
)