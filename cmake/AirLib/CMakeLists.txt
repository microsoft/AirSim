cmake_minimum_required(VERSION 3.2.2)
project(AirLib)

LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/../cmake-modules") 
INCLUDE("${CMAKE_CURRENT_LIST_DIR}/../cmake-modules/CommonSetup.cmake")
CommonSetup()

## System dependencies are found with CMake's conventions
find_package(Eigen3 3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

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
  ${RPC_LIB_INCLUDES}
)

add_library(AirLib STATIC
  ${AIRSIM_ROOT}/AirLib/src/controllers/DroneControllerBase.cpp
  ${AIRSIM_ROOT}/AirLib/src/controllers/MavLinkDroneController.cpp
  ${AIRSIM_ROOT}/AirLib/src/controllers/Settings.cpp  
  ${AIRSIM_ROOT}/AirLib/src/controllers/FileSystem.cpp  
  ${AIRSIM_ROOT}/AirLib/src/safety/ObstacleMap.cpp
  ${AIRSIM_ROOT}/AirLib/src/safety/SafetyEval.cpp
  ${AIRSIM_ROOT}/AirLib/src/rpc/RpcLibClient.cpp
  ${AIRSIM_ROOT}/AirLib/src/rpc/RpcLibServer.cpp
			)

## Add these to your app that links this library:			
target_link_libraries(${PROJECT_NAME} ${RPC_LIB})

#string(SUBSTRING ${CMAKE_STATIC_LINKER_FLAGS} 9 -1 "BUILD_PLATFORM")
#find_package(Threads REQUIRED)
#target_link_libraries(${PROJECT_NAME} ${CMAKE_THREAD_LIBS_INIT})
#target_link_libraries(${PROJECT_NAME} AirLib)
#target_link_libraries(${PROJECT_NAME} MavLinkCom)
#target_link_libraries(${PROJECT_NAME} ../AirLib/deps/rpclib/lib/${BUILD_PLATFORM}/$(Configuration)/rpc)
#IF(UNIX)
#target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})
#ENDIF()
