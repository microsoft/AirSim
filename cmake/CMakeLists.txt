cmake_minimum_required(VERSION 3.2.2)
project(AirSim)
set(RPCLIB_NAME_SUFFIX rpclib)

LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake-modules") 
INCLUDE("${CMAKE_CURRENT_LIST_DIR}/cmake-modules/CommonSetup.cmake")
CommonSetup()

add_subdirectory("${AIRSIM_ROOT}/external/rpclib" "${CMAKE_CURRENT_BINARY_DIR}/rpclib")

add_subdirectory("AirLib")
add_subdirectory("MavLinkCom")
add_subdirectory("HelloDrone")
add_subdirectory("DroneShell")
add_subdirectory("DroneServer")

