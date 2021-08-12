cmake_minimum_required(VERSION 3.5)
project(airsim_ros_pkgs)

# Add support for C++11
#if(NOT CMAKE_CXX_STANDARD)
#set(CMAKE_CXX_STANDARD 11)
#endif()

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

# rosidl_generate_interfaces(${PROJECT_NAME} "msg/GimbalAngleEulerCmd.msg"
#   "msg/GimbalAngleQuatCmd.msg" "msg/GPSYaw.msg" "msg/VelCmd.msg"
#   "msg/VelCmdGroup.msg" "msg/CarControls.msg" "msg/CarState.msg"
#   "msg/Altimeter.msg" "msg/Environment.msg" "srv/SetGPSPosition.srv"
#   "srv/Takeoff.srv" "srv/TakeoffGroup.srv" "srv/Land.srv" "srv/LandGroup.srv"
#   "srv/Reset.srv" "srv/SetLocalPosition.srv"
#   DEPENDENCIES builtin_interfaces nav_msgs geographic_msgs std_srvs
#   tf2_sensor_msgs geometry_msgs tf2_geometry_msgs sensor_msgs mavros_msgs std_msgs)



set(AIRSIM_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../../../)

add_subdirectory("${AIRSIM_ROOT}/cmake/rpclib_wrapper" rpclib_wrapper)
add_subdirectory("${AIRSIM_ROOT}/cmake/AirLib" AirLib)
add_subdirectory("${AIRSIM_ROOT}/cmake/MavLinkCom" MavLinkCom)

set(CMAKE_CXX_STANDARD 14)
set(CXX_EXP_LIB
  "-nostdinc++ -I/usr/include/c++/8 -I/usr/include/x86_64-linux-gnu/c++/8 -nodefaultlibs 
-l/usr/lib/x86_64-linux-gnu/libc++.so -l/usr/lib/x86_64-linux-gnu/libc++abi.so
-lm -lc -lgcc_s -lgcc  
-lstdc++fs -fmax-errors=10 -Wnoexcept -Wstrict-null-sentinel")
set(RPC_LIB_INCLUDES " ${AIRSIM_ROOT}/external/rpclib/rpclib-2.3.0/include")
set(RPC_LIB rpc)
message(STATUS "found RPC_LIB_INCLUDES=${RPC_LIB_INCLUDES}")

set(INCLUDE_DIRS include ${rclcpp_INCLUDE_DIRS} ${nav_msgs_INCLUDE_DIRS}
  ${geographic_msgs_INCLUDE_DIRS} ${std_srvs_INCLUDE_DIRS} ${tf2_ros_INCLUDE_DIRS}
  ${tf2_sensor_msgs_INCLUDE_DIRS} ${rclpy_INCLUDE_DIRS} ${tf2_INCLUDE_DIRS}
  ${image_transport_INCLUDE_DIRS} ${geometry_msgs_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS} ${tf2_geometry_msgs_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS} ${mavros_msgs_INCLUDE_DIRS}
  ${rosidl_default_generators_INCLUDE_DIRS} ${ament_cmake_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${AIRSIM_ROOT}/AirLib/deps/eigen3
  ${AIRSIM_ROOT}/AirLib/include
  ${RPC_LIB_INCLUDES}
  ${AIRSIM_ROOT}/MavLinkCom/include
  ${AIRSIM_ROOT}/MavLinkCom/common_utils
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${airsim_interfaces_INCLUDE_DIRS})
include_directories(${INCLUDE_DIRS})

set(LIBRARY_DIRS ${rclcpp_LIBRARY_DIRS} ${nav_msgs_LIBRARY_DIRS}
  ${geographic_msgs_LIBRARY_DIRS} ${std_srvs_LIBRARY_DIRS} ${tf2_ros_LIBRARY_DIRS}
  ${tf2_sensor_msgs_LIBRARY_DIRS} ${rclpy_LIBRARY_DIRS} ${tf2_LIBRARY_DIRS}
  ${image_transport_LIBRARY_DIRS} ${geometry_msgs_LIBRARY_DIRS}
  ${cv_bridge_LIBRARY_DIRS} ${tf2_geometry_msgs_LIBRARY_DIRS}
  ${sensor_msgs_LIBRARY_DIRS} ${mavros_msgs_LIBRARY_DIRS}
  ${rosidl_default_generators_LIBRARY_DIRS} ${ament_cmake_LIBRARY_DIRS}
  ${std_msgs_LIBRARY_DIRS}
  ${airsim_interfaces_LIBRARY_DIRS})

link_directories(${LIBRARY_DIRS})

set(LIBS ${rclcpp_LIBRARIES} ${nav_msgs_LIBRARIES} ${geographic_msgs_LIBRARIES}
  ${std_srvs_LIBRARIES} ${tf2_ros_LIBRARIES} ${tf2_sensor_msgs_LIBRARIES}
  ${rclpy_LIBRARIES} ${tf2_LIBRARIES} ${image_transport_LIBRARIES}
  ${geometry_msgs_LIBRARIES} ${cv_bridge_LIBRARIES} ${tf2_geometry_msgs_LIBRARIES}
  ${sensor_msgs_LIBRARIES} ${mavros_msgs_LIBRARIES}
  ${rosidl_default_generators_LIBRARIES} ${ament_cmake_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${airsim_interfaces_LIBRARIES})

find_package(Boost REQUIRED)
find_package(OpenCV REQUIRED)

add_library(airsim_settings_parser src/airsim_settings_parser.cpp)
target_link_libraries(airsim_settings_parser ${LIBS} AirLib)

add_library(pd_position_controller_simple src/pd_position_controller_simple.cpp)
target_link_libraries(pd_position_controller_simple ${LIBS} AirLib)


add_library(airsim_ros src/airsim_ros_wrapper.cpp)
target_link_libraries(airsim_ros ${LIBS} ${OpenCV_LIBS} yaml-cpp AirLib
  airsim_settings_parser)

add_executable(airsim_node src/airsim_node.cpp)
target_link_libraries(airsim_node airsim_ros ${LIBS} AirLib)

add_executable(pd_position_controller_simple_node
  src/pd_position_controller_simple_node.cpp)
target_link_libraries(pd_position_controller_simple_node
  pd_position_controller_simple airsim_ros ${LIBS} AirLib)

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
  airsim_ros)# ${LIBS})

ament_package()
