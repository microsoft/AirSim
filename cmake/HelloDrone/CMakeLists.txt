cmake_minimum_required(VERSION 3.2.2)
project(HelloDrone)

LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/../cmake-modules") 
INCLUDE("${CMAKE_CURRENT_LIST_DIR}/../cmake-modules/CommonSetup.cmake")
CommonSetup()

## System dependencies are found with CMake's conventions
find_package(Eigen3 3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})


IF(UNIX)
ELSE()
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /D_CONSOLE ")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /SUBSYSTEM:CONSOLE" )
ENDIF()

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  ${AIRSIM_ROOT}/HelloDrone
  ${AIRSIM_ROOT}/AirLib/include
  ${RPC_LIB_INCLUDES}
  ${AIRSIM_ROOT}/MavLinkCom/include
  ${AIRSIM_ROOT}/MavLinkCom/common_utils
)


## Declare the executable
add_executable(HelloDrone 
			${AIRSIM_ROOT}/HelloDrone/main.cpp
			)
			
target_link_libraries(${PROJECT_NAME} ${CMAKE_THREAD_LIBS_INIT} MavLinkCom AirLib c++abi ${RPC_LIB})
