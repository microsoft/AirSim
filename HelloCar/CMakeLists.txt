cmake_minimum_required(VERSION 3.5.0)
project(HelloCar)

IncludeEigen()

SetupConsoleBuild()

## Specify additional locations of header files
include_directories(
  ${AIRSIM_ROOT}/HelloCar
  ${AIRSIM_ROOT}/AirLib/include
  ${RPC_LIB_INCLUDES}
  ${AIRSIM_ROOT}/MavLinkCom/include
  ${AIRSIM_ROOT}/MavLinkCom/common_utils
)

AddExecutableSource()
			
CommonTargetLink()
target_link_libraries(${PROJECT_NAME} AirLib)
target_link_libraries(${PROJECT_NAME} ${RPC_LIB})
