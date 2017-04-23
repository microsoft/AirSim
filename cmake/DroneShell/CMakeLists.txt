cmake_minimum_required(VERSION 3.2.2)
project(DroneShell)

LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/../cmake-modules") 
INCLUDE("${CMAKE_CURRENT_LIST_DIR}/../cmake-modules/CommonSetup.cmake")
CommonSetup()

## System dependencies are found with CMake's conventions
LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/../cmake-modules") 
find_package(Eigen3 3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

IF(UNIX)
ELSE()
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /D_CONSOLE ")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /SUBSYSTEM:CONSOLE")
ENDIF()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${AIRSIM_ROOT}/DroneShell
  ${RPC_LIB_INCLUDES}
  ${AIRSIM_ROOT}/DroneShell/include;
  ${AIRSIM_ROOT}/AirLib/include
  ${AIRSIM_ROOT}/MavLinkCom/include
)


## Declare the executable
add_executable(DroneShell 
			${AIRSIM_ROOT}/DroneShell/src/main.cpp
			)
			
target_link_libraries(${PROJECT_NAME} ${CMAKE_THREAD_LIBS_INIT} AirLib MavLinkCom c++abi ${RPC_LIB})
