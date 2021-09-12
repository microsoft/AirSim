cmake_minimum_required(VERSION 3.11)
project(MavLinkCom)

include(GNUInstallDirs)

if(BUILD_TESTS)
  add_subdirectory("MavLinkTest")
endif()

LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/../cmake-modules") 
INCLUDE("${CMAKE_CURRENT_LIST_DIR}/../cmake-modules/CommonSetup.cmake")
CommonSetup()

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

add_library(${PROJECT_NAME} ${MAVLINK_SOURCES})
add_library(${SOLUTION_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

if (WIN32)
	target_compile_definitions(${PROJECT_NAME} PRIVATE
		-D_UNICODE -DUNICODE
	)
endif()
target_include_directories(${PROJECT_NAME} 
  PUBLIC
    $<BUILD_INTERFACE:${AIRSIM_ROOT}/MavLinkCom/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/${SOLUTION_NAME}/${PROJECT_NAME}>
  PRIVATE
    $<BUILD_INTERFACE:
      ${AIRSIM_ROOT}/MavLinkCom
      ${AIRSIM_ROOT}/MavLinkCom/common_utils  
    >
)

set_target_properties(${PROJECT_NAME} PROPERTIES
  WINDOWS_EXPORT_ALL_SYMBOLS ON
)

if(BUILD_TESTS)
  add_subdirectory("MavLinkTest")
endif()

install(DIRECTORY "${AIRSIM_ROOT}/${PROJECT_NAME}/include/" DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/${SOLUTION_NAME}/${PROJECT_NAME}")
install(TARGETS ${PROJECT_NAME}
	EXPORT ${SOLUTION_NAME}-export
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)