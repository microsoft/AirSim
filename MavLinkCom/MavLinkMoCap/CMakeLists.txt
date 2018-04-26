cmake_minimum_required(VERSION 3.2.2)
project(MavLinkMoCap)

SetupConsoleBuild()

include_directories(
  ${AIRSIM_ROOT}/MavLinkCom/MavLinkTest
  ${AIRSIM_ROOT}/MavLinkCom/include
  ${AIRSIM_ROOT}/MavLinkCom/common_utils
)

GetLocalSources(PROJECT_SOURCE_FILES .)

if (EXISTS "NPTrackingTools.lib") 
    add_executable(MavLinkMoCap ${PROJECT_SOURCE_FILES})			
    target_link_libraries(MavLinkTest ${CMAKE_THREAD_LIBS_INIT} NPTrackingTools.lib MavLinkCom ${CXX_EXP_LIB})
endif()
