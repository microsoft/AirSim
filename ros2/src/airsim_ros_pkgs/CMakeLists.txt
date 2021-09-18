cmake_minimum_required(VERSION 3.5)
project(airsim_ros_pkgs)

find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(tf2 REQUIRED)
find_package(image_transport REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
find_package(airsim_interfaces REQUIRED)
find_package(OpenCV REQUIRED)

set(AIRSIM_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../../../)

add_subdirectory("${AIRSIM_ROOT}/cmake/rpclib_wrapper" rpclib_wrapper)
add_subdirectory("${AIRSIM_ROOT}/cmake/AirLib" AirLib)
add_subdirectory("${AIRSIM_ROOT}/cmake/MavLinkCom" MavLinkCom)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "-O3 -Wall -Wextra -Wnoexcept -Wstrict-null-sentinel")
set(CXX_EXP_LIB "-nostdinc++ -I/usr/include/c++/8 -I/usr/include/x86_64-linux-gnu/c++/8 -nodefaultlibs 
-l/usr/lib/x86_64-linux-gnu/libc++.so -l/usr/lib/x86_64-linux-gnu/libc++abi.so
-lm -lc -lgcc_s -lgcc
-lstdc++fs -fmax-errors=10")
set(RPC_LIB_INCLUDES " ${AIRSIM_ROOT}/external/rpclib/rpclib-2.3.0/include")
set(RPC_LIB rpc)
message(STATUS "found RPC_LIB_INCLUDES=${RPC_LIB_INCLUDES}")

set(INCLUDE_DIRS include 
  ${AIRSIM_ROOT}/AirLib/deps/eigen3
  ${AIRSIM_ROOT}/AirLib/include
  ${RPC_LIB_INCLUDES}
  ${AIRSIM_ROOT}/MavLinkCom/include
  ${AIRSIM_ROOT}/MavLinkCom/common_utils
  ${OpenCV_INCLUDE_DIRS}
  )
include_directories(${INCLUDE_DIRS})

add_library(airsim_settings_parser src/airsim_settings_parser.cpp)
target_link_libraries(airsim_settings_parser AirLib)

add_library(pd_position_controller_simple src/pd_position_controller_simple.cpp)
target_link_libraries(pd_position_controller_simple AirLib)
ament_target_dependencies(pd_position_controller_simple 
  rclcpp
  nav_msgs
  geometry_msgs
  airsim_interfaces
)

add_library(airsim_ros src/airsim_ros_wrapper.cpp)
target_link_libraries(airsim_ros ${OpenCV_LIBS} yaml-cpp AirLib airsim_settings_parser)
ament_target_dependencies(airsim_ros 
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  image_transport
  tf2_ros
  cv_bridge
  airsim_interfaces
)

add_executable(airsim_node src/airsim_node.cpp)
target_link_libraries(airsim_node airsim_ros AirLib)
ament_target_dependencies(airsim_node 
  rclcpp
)

add_executable(pd_position_controller_simple_node
  src/pd_position_controller_simple_node.cpp)
target_link_libraries(pd_position_controller_simple_node
  pd_position_controller_simple airsim_ros AirLib)
ament_target_dependencies(pd_position_controller_simple_node 
  rclcpp
)

# rosidl_target_interfaces(airsim_node
# ${PROJECT_NAME} "rosidl_typesupport_cpp")

# rosidl_target_interfaces(pd_position_controller_simple_node
#   ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS airsim_node pd_position_controller_simple_node
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS airsim_ros pd_position_controller_simple
  ARCHIVE
  DESTINATION lib
  LIBRARY
  DESTINATION lib)

install(FILES README.md DESTINATION share/${PROJECT_NAME})

install(DIRECTORY launch rviz DESTINATION share/${PROJECT_NAME})



ament_export_dependencies(rclcpp)
ament_export_dependencies(nav_msgs)
ament_export_dependencies(geographic_msgs)
ament_export_dependencies(std_srvs)
ament_export_dependencies(tf2_ros)
ament_export_dependencies(tf2_sensor_msgs)
ament_export_dependencies(rclpy)
ament_export_dependencies(tf2)
ament_export_dependencies(image_transport)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(cv_bridge)
ament_export_dependencies(tf2_geometry_msgs)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(mavros_msgs)
ament_export_dependencies(rosidl_default_generators)
ament_export_dependencies(ament_cmake)
ament_export_dependencies(std_msgs)
ament_export_dependencies(rosidl_default_runtime)
ament_export_dependencies(airsim_interfaces)

ament_export_include_directories(${INCLUDE_DIRS})
ament_export_libraries(airsim_settings_parser pd_position_controller_simple
  airsim_ros)

ament_package()
