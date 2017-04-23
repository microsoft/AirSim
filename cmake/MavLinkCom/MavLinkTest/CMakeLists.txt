cmake_minimum_required(VERSION 3.2.2)
project(MavLinkTest)

LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/../../cmake-modules") 
INCLUDE("${CMAKE_CURRENT_LIST_DIR}/../../cmake-modules/CommonSetup.cmake")
CommonSetup()

IF(UNIX)
ELSE()
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /D_CONSOLE ")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /SUBSYSTEM:CONSOLE ")
ENDIF()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${AIRSIM_ROOT}/MavLinkCom/MavLinkTest
  ${AIRSIM_ROOT}/MavLinkCom/include
  ${AIRSIM_ROOT}/MavLinkCom/common_utils
)

## Declare the executable
add_executable(MavLinkTest 
			${AIRSIM_ROOT}/MavLinkCom/MavLinkTest/Commands.cpp
			${AIRSIM_ROOT}/MavLinkCom/MavLinkTest/UnitTests.cpp
			${AIRSIM_ROOT}/MavLinkCom/MavLinkTest/main.cpp
			)
			
target_link_libraries(${PROJECT_NAME} ${CMAKE_THREAD_LIBS_INIT} MavLinkCom c++abi ${CXX_EXP_LIB})
