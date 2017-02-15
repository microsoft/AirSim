cmake_minimum_required(VERSION 2.8.3)
project(MavLinkCom)

LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/../cmake-modules") 
INCLUDE("${CMAKE_CURRENT_LIST_DIR}/../cmake-modules/CommonSetup.cmake")
CommonSetup()

message(STATUS "Boost_INCLUDE_DIR=${Boost_INCLUDE_DIR}")

## System dependencies are found with CMake's conventions
# SET(Boost_DEBUG  TRUE)
find_package(Boost REQUIRED COMPONENTS system)

if(Boost_FOUND)
	
endif()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${AIRSIM_ROOT}/MavLinkCom
  ${AIRSIM_ROOT}/MavLinkCom/common_utils
  ${AIRSIM_ROOT}/MavLinkCom/include
  ${Boost_INCLUDE_DIRS}
)


LIST(APPEND MAVLINK_SOURCES "${AIRSIM_ROOT}/MavLinkCom/src/MavLinkConnection.cpp") 
LIST(APPEND MAVLINK_SOURCES "${AIRSIM_ROOT}/MavLinkCom/src/MavLinkFtpClient.cpp") 
LIST(APPEND MAVLINK_SOURCES "${AIRSIM_ROOT}/MavLinkCom/src/MavLinkLog.cpp") 
LIST(APPEND MAVLINK_SOURCES "${AIRSIM_ROOT}/MavLinkCom/src/MavLinkMessageBase.cpp") 
LIST(APPEND MAVLINK_SOURCES "${AIRSIM_ROOT}/MavLinkCom/src/MavLinkMessages.cpp") 
LIST(APPEND MAVLINK_SOURCES "${AIRSIM_ROOT}/MavLinkCom/src/MavLinkNode.cpp") 	
LIST(APPEND MAVLINK_SOURCES "${AIRSIM_ROOT}/MavLinkCom/src/MavLinkTcpServer.cpp") 
LIST(APPEND MAVLINK_SOURCES "${AIRSIM_ROOT}/MavLinkCom/src/MavLinkVehicle.cpp") 
LIST(APPEND MAVLINK_SOURCES "${AIRSIM_ROOT}/MavLinkCom/src/MavLinkVideoStream.cpp") 
LIST(APPEND MAVLINK_SOURCES "${AIRSIM_ROOT}/MavLinkCom/src/MavLinkSemaphore.cpp") 
LIST(APPEND MAVLINK_SOURCES "${AIRSIM_ROOT}/MavLinkCom/src/impl/MavLinkConnectionImpl.cpp") 
LIST(APPEND MAVLINK_SOURCES "${AIRSIM_ROOT}/MavLinkCom/src/impl/MavLinkFtpClientImpl.cpp") 
LIST(APPEND MAVLINK_SOURCES "${AIRSIM_ROOT}/MavLinkCom/src/impl/MavLinkNodeImpl.cpp") 
LIST(APPEND MAVLINK_SOURCES "${AIRSIM_ROOT}/MavLinkCom/src/impl/MavLinkTcpServerImpl.cpp") 
LIST(APPEND MAVLINK_SOURCES "${AIRSIM_ROOT}/MavLinkCom/src/impl/MavLinkVehicleImpl.cpp") 
LIST(APPEND MAVLINK_SOURCES "${AIRSIM_ROOT}/MavLinkCom/src/impl/MavLinkVideoStreamImpl.cpp") 
LIST(APPEND MAVLINK_SOURCES "${AIRSIM_ROOT}/MavLinkCom/src/serial_com/SerialPort.cpp") 
LIST(APPEND MAVLINK_SOURCES "${AIRSIM_ROOT}/MavLinkCom/src/serial_com/TcpServer.cpp") 
LIST(APPEND MAVLINK_SOURCES "${AIRSIM_ROOT}/MavLinkCom/src/serial_com/TcpClientPort.cpp") 
LIST(APPEND MAVLINK_SOURCES "${AIRSIM_ROOT}/MavLinkCom/src/serial_com/UdpClientPort.cpp") 
LIST(APPEND MAVLINK_SOURCES "${AIRSIM_ROOT}/MavLinkCom/src/serial_com/UdpServer.cpp")

IF(UNIX)
    LIST(APPEND MAVLINK_SOURCES "${AIRSIM_ROOT}/MavLinkCom/src/impl/linux/MavLinkFindSerialPorts.cpp")
ELSE()
    LIST(APPEND MAVLINK_SOURCES "${AIRSIM_ROOT}/MavLinkCom/src/impl/windows/MavLinkFindSerialPorts.cpp")
ENDIF()

add_library(MavLinkCom STATIC ${MAVLINK_SOURCES})

## Add these to your app that links this library:			
# find_package(Threads REQUIRED)
# target_link_libraries (${PROJECT_NAME} ${CMAKE_THREAD_LIBS_INIT})
# target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})
