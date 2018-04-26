cmake_minimum_required(VERSION 3.5.0)
project(DroneServer)
set(tool_name DroneServer)

IncludeEigen()

SetupConsoleBuild()


include_directories(
  ${AIRSIM_ROOT}/DroneServer
  ${AIRSIM_ROOT}/MavLinkCom/include
  ${RPC_LIB_INCLUDES}
  ${AIRSIM_ROOT}/AirLib/include
)

GetLocalSources(PROJECT_SOURCE_FILES .)

add_executable(${tool_name} ${PROJECT_SOURCE_FILES})
			
CommonTargetLink()
target_link_libraries(${PROJECT_NAME} AirLib)
target_link_libraries(${PROJECT_NAME} MavLinkCom)
target_link_libraries(${PROJECT_NAME} ${RPC_LIB})
