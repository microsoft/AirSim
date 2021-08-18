cmake_minimum_required(VERSION 3.10.0)
project(airsim_ros_pkgs)

# set this to path to AirSim root folder if you want your catkin workspace in a custom directory
set(AIRSIM_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../../../)

add_subdirectory("${AIRSIM_ROOT}/cmake" AirSim)

set(CMAKE_CXX_STANDARD 11)
set(CXX_EXP_LIB "-nostdinc++ -I/usr/include/c++/8 -I/usr/include/x86_64-linux-gnu/c++/8 -nodefaultlibs 
-l/usr/lib/x86_64-linux-gnu/libc++.so -l/usr/lib/x86_64-linux-gnu/libc++abi.so
-lm -lc -lgcc_s -lgcc  
-lstdc++fs -fmax-errors=10 -Wnoexcept -Wstrict-null-sentinel") 

find_package(yaml-cpp CONFIG REQUIRED)
find_package(rpclib CONFIG REQUIRED)
find_package(Eigen3 CONFIG REQUIRED) 
find_package(Boost REQUIRED)
find_package(OpenCV REQUIRED)

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
  geographic_msgs
  geometry_msgs
  std_srvs
  tf2
  tf2_ros
  tf2_sensor_msgs
  tf2_geometry_msgs
)

add_message_files(
  FILES
  GimbalAngleEulerCmd.msg
  GimbalAngleQuatCmd.msg
  GPSYaw.msg
  VelCmd.msg
  VelCmdGroup.msg
  CarControls.msg
  CarState.msg
  Altimeter.msg
  Environment.msg
)

add_service_files(
  FILES
  SetGPSPosition.srv
  Takeoff.srv
  TakeoffGroup.srv
  Land.srv
  LandGroup.srv
  Reset.srv
  SetLocalPosition.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  geographic_msgs
)

catkin_package(
  INCLUDE_DIRS include
  # LIBRARIES airsim_ros
  CATKIN_DEPENDS message_runtime roscpp std_msgs # nodelet 
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_library(airsim_settings_parser src/airsim_settings_parser.cpp)
target_link_libraries(airsim_settings_parser ${catkin_LIBRARIES} AirSim::AirLib)

add_library(pd_position_controller_simple src/pd_position_controller_simple.cpp)
add_dependencies(pd_position_controller_simple ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(pd_position_controller_simple ${catkin_EXPORTED_TARGETS})
target_link_libraries(pd_position_controller_simple AirSim::AirLib)

add_library(airsim_ros src/airsim_ros_wrapper.cpp)
add_dependencies(airsim_ros ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(airsim_ros ${catkin_LIBRARIES} ${OpenCV_LIBS} yaml-cpp AirLib airsim_settings_parser)

add_executable(airsim_node src/airsim_node.cpp)
target_link_libraries(airsim_node airsim_ros ${catkin_LIBRARIES} AirSim::AirLib)

add_executable(pd_position_controller_simple_node src/pd_position_controller_simple_node.cpp)
target_link_libraries(pd_position_controller_simple_node pd_position_controller_simple airsim_ros ${catkin_LIBRARIES} AirSim::AirLib)

install(TARGETS
    #list of nodes
    airsim_node
    pd_position_controller_simple_node
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS 
    #list of shared libraries
    airsim_ros
    pd_position_controller_simple
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES 
  #list of necessary files (xml...)
  README.md
  # settings.json
  # nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# for shipping binaries. run `$ catin_make install`
install(DIRECTORY
  launch
  rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  # PATTERN ".git" EXCLUDE
)
