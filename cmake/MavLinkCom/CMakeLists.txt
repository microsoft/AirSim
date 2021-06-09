cmake_minimum_required(VERSION 3.5.0)
project(MavLinkCom)

add_subdirectory("MavLinkTest")

LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/../cmake-modules") 
INCLUDE("${CMAKE_CURRENT_LIST_DIR}/../cmake-modules/CommonSetup.cmake")
CommonSetup()

include_directories(
  ${AIRSIM_ROOT}/MavLinkCom
  ${AIRSIM_ROOT}/MavLinkCom/common_utils
  ${AIRSIM_ROOT}/MavLinkCom/include
)


LIST(APPEND MAVLINK_SOURCES "${AIRSIM_ROOT}/MavLinkCom/common_utils/FileSystem.cpp")
LIST(APPEND MAVLINK_SOURCES "${AIRSIM_ROOT}/MavLinkCom/common_utils/ThreadUtils.cpp")
LIST(APPEND MAVLINK_SOURCES "${AIRSIM_ROOT}/MavLinkCom/src/AdHocConnection.cpp") 
LIST(APPEND MAVLINK_SOURCES "${AIRSIM_ROOT}/MavLinkCom/src/MavLinkConnection.cpp") 
LIST(APPEND MAVLINK_SOURCES "${AIRSIM_ROOT}/MavLinkCom/src/MavLinkFtpClient.cpp") 
LIST(APPEND MAVLINK_SOURCES "${AIRSIM_ROOT}/MavLinkCom/src/MavLinkLog.cpp") 
LIST(APPEND MAVLINK_SOURCES "${AIRSIM_ROOT}/MavLinkCom/src/MavLinkMessageBase.cpp") 
LIST(APPEND MAVLINK_SOURCES "${AIRSIM_ROOT}/MavLinkCom/src/MavLinkMessages.cpp") 
LIST(APPEND MAVLINK_SOURCES "${AIRSIM_ROOT}/MavLinkCom/src/MavLinkNode.cpp") 	
LIST(APPEND MAVLINK_SOURCES "${AIRSIM_ROOT}/MavLinkCom/src/MavLinkTcpServer.cpp") 
LIST(APPEND MAVLINK_SOURCES "${AIRSIM_ROOT}/MavLinkCom/src/MavLinkVehicle.cpp") 
LIST(APPEND MAVLINK_SOURCES "${AIRSIM_ROOT}/MavLinkCom/src/MavLinkVideoStream.cpp") 
LIST(APPEND MAVLINK_SOURCES "${AIRSIM_ROOT}/MavLinkCom/src/Semaphore.cpp") 
LIST(APPEND MAVLINK_SOURCES "${AIRSIM_ROOT}/MavLinkCom/src/UdpSocket.cpp") 
LIST(APPEND MAVLINK_SOURCES "${AIRSIM_ROOT}/MavLinkCom/src/impl/AdHocConnectionImpl.cpp") 
LIST(APPEND MAVLINK_SOURCES "${AIRSIM_ROOT}/MavLinkCom/src/impl/MavLinkConnectionImpl.cpp") 
LIST(APPEND MAVLINK_SOURCES "${AIRSIM_ROOT}/MavLinkCom/src/impl/MavLinkFtpClientImpl.cpp") 
LIST(APPEND MAVLINK_SOURCES "${AIRSIM_ROOT}/MavLinkCom/src/impl/MavLinkNodeImpl.cpp") 
LIST(APPEND MAVLINK_SOURCES "${AIRSIM_ROOT}/MavLinkCom/src/impl/MavLinkTcpServerImpl.cpp") 
LIST(APPEND MAVLINK_SOURCES "${AIRSIM_ROOT}/MavLinkCom/src/impl/MavLinkVehicleImpl.cpp") 
LIST(APPEND MAVLINK_SOURCES "${AIRSIM_ROOT}/MavLinkCom/src/impl/MavLinkVideoStreamImpl.cpp") 
LIST(APPEND MAVLINK_SOURCES "${AIRSIM_ROOT}/MavLinkCom/src/impl/UdpSocketImpl.cpp") 
LIST(APPEND MAVLINK_SOURCES "${AIRSIM_ROOT}/MavLinkCom/src/serial_com/SerialPort.cpp") 
LIST(APPEND MAVLINK_SOURCES "${AIRSIM_ROOT}/MavLinkCom/src/serial_com/TcpClientPort.cpp") 
LIST(APPEND MAVLINK_SOURCES "${AIRSIM_ROOT}/MavLinkCom/src/serial_com/UdpClientPort.cpp") 
LIST(APPEND MAVLINK_SOURCES "${AIRSIM_ROOT}/MavLinkCom/src/serial_com/SocketInit.cpp")
LIST(APPEND MAVLINK_SOURCES "${AIRSIM_ROOT}/MavLinkCom/src/serial_com/wifi.cpp")

IF(UNIX)
    LIST(APPEND MAVLINK_SOURCES "${AIRSIM_ROOT}/MavLinkCom/src/impl/linux/MavLinkFindSerialPorts.cpp")
ELSE()
    LIST(APPEND MAVLINK_SOURCES "${AIRSIM_ROOT}/MavLinkCom/src/impl/windows/WindowsFindSerialPorts.cpp")
ENDIF()



add_library(MavLinkCom STATIC ${MAVLINK_SOURCES})

CommonTargetLink()

