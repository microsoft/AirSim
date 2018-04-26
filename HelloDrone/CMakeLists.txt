cmake_minimum_required(VERSION 3.5.0)
project(HelloDrone)
set(tool_name HelloDrone)

IncludeEigen()

SetupConsoleBuild()

## Specify additional locations of header files
include_directories(
  ${AIRSIM_ROOT}/HelloDrone
  ${AIRSIM_ROOT}/AirLib/include
  ${RPC_LIB_INCLUDES}
  ${AIRSIM_ROOT}/MavLinkCom/include
  ${AIRSIM_ROOT}/MavLinkCom/common_utils
)

GetLocalSources(PROJECT_SOURCE_FILES .)

add_executable(${tool_name} ${PROJECT_SOURCE_FILES})
			
CommonTargetLink()
target_link_libraries(${PROJECT_NAME} AirLib)
target_link_libraries(${PROJECT_NAME} ${RPC_LIB})
