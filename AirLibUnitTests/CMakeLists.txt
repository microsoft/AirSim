cmake_minimum_required(VERSION 3.5.0)
project(AirLibUnitTests)

IncludeEigen()

SetupConsoleBuild()

include_directories(
  ${AIRSIM_ROOT}/AirLibUnitTests
  ${AIRSIM_ROOT}/AirLib/include
  ${AIRSIM_ROOT}/MavLinkCom/include
)

AddExecutableSource()

CommonTargetLink()
target_link_libraries(${PROJECT_NAME} AirLib)
target_link_libraries(${PROJECT_NAME} MavLinkCom)
target_link_libraries(${PROJECT_NAME} ${RPC_LIB})
