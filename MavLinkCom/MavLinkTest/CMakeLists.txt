cmake_minimum_required(VERSION 3.2.2)
project(MavLinkTest)

SetupConsoleBuild()

include_directories(
  ${AIRSIM_ROOT}/MavLinkCom/MavLinkTest
  ${AIRSIM_ROOT}/MavLinkCom/include
  ${AIRSIM_ROOT}/MavLinkCom/common_utils
)

set(include
    Commands.h
    ConsoleBase.h
    UnitTests.h
)

set(src
    Commands.cpp
    main.cpp
    UnitTests.cpp
)

source_group("src" FILES ${src})
source_group("include" FILES ${include})

add_executable(MavLinkTest ${src}  ${include})
			
target_link_libraries(MavLinkTest ${CMAKE_THREAD_LIBS_INIT} MavLinkCom ${CXX_EXP_LIB})
