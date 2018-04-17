cmake_minimum_required(VERSION 3.5.0)
project(MavLinkCom)

add_subdirectory("MavLinkTest")

include_directories(
  ${AIRSIM_ROOT}/MavLinkCom
  ${AIRSIM_ROOT}/MavLinkCom/common_utils
  ${AIRSIM_ROOT}/MavLinkCom/include
)

set(src
    common_utils/FileSystem.cpp
    common_utils/ThreadUtils.cpp
    src/MavLinkConnection.cpp
    src/MavLinkFtpClient.cpp
    src/MavLinkLog.cpp
    src/MavLinkMessageBase.cpp
    src/MavLinkMessages.cpp
    src/MavLinkNode.cpp
    src/MavLinkTcpServer.cpp
    src/MavLinkVehicle.cpp
    src/MavLinkVideoStream.cpp
    src/Semaphore.cpp
    src/impl/MavLinkConnectionImpl.cpp
    src/impl/MavLinkFtpClientImpl.cpp
    src/impl/MavLinkNodeImpl.cpp
    src/impl/MavLinkTcpServerImpl.cpp
    src/impl/MavLinkVehicleImpl.cpp
    src/impl/MavLinkVideoStreamImpl.cpp
    src/serial_com/SerialPort.cpp
    src/serial_com/TcpClientPort.cpp
    src/serial_com/UdpClientPort.cpp
    src/serial_com/SocketInit.cpp
    src/serial_com/wifi.cpp 
)

set(include
    include/AsyncResult.hpp
    include/MavLinkConnection.hpp
    include/MavLinkFtpClient.hpp
    include/MavLinkLog.hpp
    include/MavLinkMessageBase.hpp
    include/MavLinkMessages.hpp
    include/MavLinkNode.hpp
    include/MavLinkTcpServer.hpp
    include/MavLinkVehicle.hpp
    include/MavLinkVideoStream.hpp
    include/Semaphore.hpp
    include/VehicleState.hpp
)
set(mavlink
    mavlink/checksum.h
    mavlink/common/common.h
    mavlink/common/mavlink.h
    mavlink/common/version.h
    mavlink/mavlink_conversions.h
    mavlink/mavlink_get_info.h
    mavlink/mavlink_helpers.h
    mavlink/mavlink_sha256.h
    mavlink/mavlink_types.h
    mavlink/protocol.h
)

IF(UNIX)
    LIST(APPEND src "src/impl/linux/MavLinkFindSerialPorts.cpp")
ELSE()
    LIST(APPEND src "src/impl/windows/WindowsFindSerialPorts.cpp")
ENDIF()

source_group("src" FILES ${src})
source_group("include" FILES ${include})
source_group("mavlink" FILES ${mavlink})

add_library(MavLinkCom STATIC ${src} ${include} ${mavlink})

CommonTargetLink()

