cmake_minimum_required(VERSION 3.3.2)
project(MavLinkTest)

LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/../../cmake-modules") 
INCLUDE("${CMAKE_CURRENT_LIST_DIR}/../../cmake-modules/CommonSetup.cmake")
CommonSetup()

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system filesystem )

IF(UNIX)
ELSE()
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /D_CONSOLE ")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /SUBSYSTEM:CONSOLE /LIBPATH:${Boost_INCLUDE_DIRS}/stage/lib")
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
  ${Boost_INCLUDE_DIRS}
)

## Declare the executable
add_executable(MavLinkTest 
			${AIRSIM_ROOT}/MavLinkCom/MavLinkTest/Commands.cpp
			${AIRSIM_ROOT}/MavLinkCom/MavLinkTest/UnitTests.cpp
			${AIRSIM_ROOT}/MavLinkCom/MavLinkTest/main.cpp
			)
			
find_package(Threads REQUIRED)
target_link_libraries(${PROJECT_NAME} ${CMAKE_THREAD_LIBS_INIT})
target_link_libraries(${PROJECT_NAME} MavLinkCom)
IF(UNIX)
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})
ENDIF()
