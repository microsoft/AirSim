cmake_minimum_required(VERSION 2.8.3)
project(AirLib)

LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/../cmake-modules") 
INCLUDE("${CMAKE_CURRENT_LIST_DIR}/../cmake-modules/CommonSetup.cmake")
CommonSetup()

## System dependencies are found with CMake's conventions
find_package(Eigen3 3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

message(STATUS "TEST=${CMAKE_CURRENT_LIST_DIR}/${AIRSIM_ROOT}/AirLib/src/control/DroneControlBase.cpp")

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${AIRSIM_ROOT}/AirLib/
  ${AIRSIM_ROOT}/AirLib/include
  ${AIRSIM_ROOT}/MavLinkCom/include
  ${AIRSIM_ROOT}/AirLib//deps/rpclib/include
)

add_library(AirLib STATIC
  ${AIRSIM_ROOT}/AirLib/src/control/DroneControlBase.cpp
  ${AIRSIM_ROOT}/AirLib/src/control/MavLinkDroneControl.cpp
  ${AIRSIM_ROOT}/AirLib/src/control/MavLinkHelper.cpp
  ${AIRSIM_ROOT}/AirLib/src/control/ObstacleMap.cpp
  ${AIRSIM_ROOT}/AirLib/src/control/RpcLibClient.cpp
  ${AIRSIM_ROOT}/AirLib/src/control/RpcLibServer.cpp
  ${AIRSIM_ROOT}/AirLib/src/control/SafetyEval.cpp
			)

## Add these to your app that links this library:			

#string(SUBSTRING ${CMAKE_STATIC_LINKER_FLAGS} 9 -1 "BUILD_PLATFORM")
#find_package(Threads REQUIRED)
#target_link_libraries(${PROJECT_NAME} ${CMAKE_THREAD_LIBS_INIT})
#target_link_libraries(${PROJECT_NAME} AirLib)
#target_link_libraries(${PROJECT_NAME} MavLinkCom)
#target_link_libraries(${PROJECT_NAME} ../AirLib/deps/rpclib/lib/${BUILD_PLATFORM}/$(Configuration)/rpc)
#IF(UNIX)
#target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})
#ENDIF()
