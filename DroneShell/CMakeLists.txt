cmake_minimum_required(VERSION 3.5.0)
project(DroneShell)
set(tool_name DroneShell)

IncludeEigen()

SetupConsoleBuild()

set(src
    src/main.cpp
)

set(include
    include/linenoise.hpp
    include/SimpleSHell.hpp
)

source_group("src" FILES ${src})
source_group("include" FILES ${include})

include_directories(
  ${AIRSIM_ROOT}/DroneShell
  ${RPC_LIB_INCLUDES}
  ${AIRSIM_ROOT}/DroneShell/include;
  ${AIRSIM_ROOT}/AirLib/include
  ${AIRSIM_ROOT}/MavLinkCom/include
)

add_executable(${tool_name} ${src} ${include})
			
CommonTargetLink()
target_link_libraries(${PROJECT_NAME} AirLib)
target_link_libraries(${PROJECT_NAME} ${RPC_LIB})
