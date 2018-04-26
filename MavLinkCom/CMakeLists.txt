cmake_minimum_required(VERSION 3.5.0)
project(MavLinkCom)

add_subdirectory("MavLinkTest")

include_directories(
  ${AIRSIM_ROOT}/MavLinkCom
  ${AIRSIM_ROOT}/MavLinkCom/common_utils
  ${AIRSIM_ROOT}/MavLinkCom/include
)

GetLocalSources(PROJECT_SOURCE_FILES include)
GetLocalSources(PROJECT_SOURCE_FILES src)
GetLocalSources(PROJECT_SOURCE_FILES common_utils)
GetLocalSources(PROJECT_SOURCE_FILES mavlink)

IF(UNIX)
    LIST(REMOVE_ITEM PROJECT_SOURCE_FILES "${AIRSIM_ROOT}/MavLinkCom/src/impl/windows/WindowsFindSerialPorts.cpp")
ELSE()
    LIST(REMOVE_ITEM PROJECT_SOURCE_FILES "${AIRSIM_ROOT}/MavLinkCom/src/impl/linux/MavLinkFindSerialPorts.cpp")
ENDIF()

add_library(MavLinkCom STATIC ${PROJECT_SOURCE_FILES})

CommonTargetLink()

