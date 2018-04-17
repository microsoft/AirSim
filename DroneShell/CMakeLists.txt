cmake_minimum_required(VERSION 3.5.0)
project(DroneShell)

IncludeEigen()

SetupConsoleBuild()

include_directories(
  ${AIRSIM_ROOT}/DroneShell
  ${RPC_LIB_INCLUDES}
  ${AIRSIM_ROOT}/DroneShell/include;
  ${AIRSIM_ROOT}/AirLib/include
  ${AIRSIM_ROOT}/MavLinkCom/include
)

AddExecutableSource()
			
CommonTargetLink()
target_link_libraries(${PROJECT_NAME} AirLib)
target_link_libraries(${PROJECT_NAME} ${RPC_LIB})
