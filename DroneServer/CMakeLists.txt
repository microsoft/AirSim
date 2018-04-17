cmake_minimum_required(VERSION 3.5.0)
project(DroneServer)
set(tool_name DroneServer)

IncludeEigen()

SetupConsoleBuild()


set(src
    main.cpp
)

set(include
)

source_group("src" FILES ${src})
source_group("include" FILES ${include})

include_directories(
  ${AIRSIM_ROOT}/DroneServer
  ${AIRSIM_ROOT}/MavLinkCom/include
  ${RPC_LIB_INCLUDES}
  ${AIRSIM_ROOT}/AirLib/include
)

add_executable(${tool_name} ${src} ${include})
			
CommonTargetLink()
target_link_libraries(${PROJECT_NAME} AirLib)
target_link_libraries(${PROJECT_NAME} MavLinkCom)
target_link_libraries(${PROJECT_NAME} ${RPC_LIB})
