cmake_minimum_required(VERSION 3.5.0)
project(DroneShell)
set(tool_name DroneShell)

IncludeEigen()

SetupConsoleBuild()

include_directories(
  ${AIRSIM_ROOT}/DroneShell
  ${RPC_LIB_INCLUDES}
  ${AIRSIM_ROOT}/DroneShell/include;
  ${AIRSIM_ROOT}/AirLib/include
  ${AIRSIM_ROOT}/MavLinkCom/include
)

GetLocalSources(PROJECT_SOURCE_FILES .)

add_executable(${tool_name} ${PROJECT_SOURCE_FILES})
			
CommonTargetLink()
target_link_libraries(${PROJECT_NAME} AirLib)
target_link_libraries(${PROJECT_NAME} ${RPC_LIB})
