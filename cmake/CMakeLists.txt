cmake_minimum_required(VERSION 3.2.2)
project(AirSim)

add_subdirectory("{AIRSIM_ROOT}/external/rpclib")
ADD_CUSTOM_COMMAND(TARGET rpc
          POST_BUILD
          COMMAND ${CMAKE_COMMAND} -E make_directory "${AIRSIM_ROOT}/AirLib/deps/rpclib/lib/x64/linux"
          COMMAND ${CMAKE_COMMAND} -E copy_if_different ${RPC_BIN_NAME} "${AIRSIM_ROOT}/AirLib/deps/rpclib/lib/x64/linux/."
          COMMAND ${CMAKE_COMMAND} -E make_directory "${AIRSIM_ROOT}/AirLib/deps/rpclib/include"
          COMMAND ${CMAKE_COMMAND} -E copy_if_different "{AIRSIM_ROOT}/external/rpclib/include/" "${AIRSIM_ROOT}/AirLib/deps/rpclib/include/"
)

add_subdirectory("AirLib")
add_subdirectory("MavLinkCom")
add_subdirectory("HelloDrone")
add_subdirectory("DroneShell")
add_subdirectory("DroneServer")
