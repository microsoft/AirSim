cmake_minimum_required(VERSION 3.5.0)

if (${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    set(MACOSX TRUE)
endif()

find_path(AIRSIM_ROOT NAMES AirSim.sln PATHS ".." "../.." "../../.." "../../../.." "../../../../.." "../../../../../..")
message(AirSim Root directory: ${AIRSIM_ROOT})

LIST(APPEND CMAKE_MODULE_PATH "${AIRSIM_ROOT}/cmake/cmake-modules")
LIST(APPEND CMAKE_MODULE_PATH "${RPC_SOURCE_DIR}/cmake")
LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")


INCLUDE("${AIRSIM_ROOT}/cmake/cmake-modules/CommonSetup.cmake")
INCLUDE("${CMAKE_CURRENT_SOURCE_DIR}/cmake/rpc-setup.cmake")
INCLUDE("${CMAKE_CURRENT_SOURCE_DIR}/cmake/mav-setup.cmake")
INCLUDE("${CMAKE_CURRENT_SOURCE_DIR}/cmake/airlib-setup.cmake")
INCLUDE("${CMAKE_CURRENT_SOURCE_DIR}/cmake/airsimwrapper-setup.cmake")

IncludeEigen()

project(AirsimWrapper VERSION 0)

find_package(Boost REQUIRED COMPONENTS filesystem)
if(Boost_FOUND)
    MESSAGE("Boost information:") 
    MESSAGE("  Boost_INCLUDE_DIRS: ${Boost_INCLUDE_DIRS}") 
    MESSAGE("  Boost_LIBRARIES: ${Boost_LIBRARIES}") 
    MESSAGE("  Boost_LIBRARY_DIRS: ${Boost_LIBRARY_DIRS}") 
endif()

include_directories(${Boost_INCLUDE_DIRS})

# RPC includes & source files
BuildRpc()
# MavLink source files
BuildMavLink()
#AirLib source files
BuildAirLib()
#AirsimWrapper source files
BuildAirsimWrapper()


###################### Link source files to library ######################################
if (${APPLE})
	add_library(
	    ${PROJECT_NAME} MODULE
	    ${RPC_LIBRARY_SOURCE_FILES}
	    ${MAVLINK_LIBRARY_SOURCE_FILES}
	    ${AIRLIB_LIBRARY_SOURCE_FILES}
	    ${AIRSIMWRAPPER_LIBRARY_SOURCE_FILES}
	)

	set_target_properties(${PROJECT_NAME} PROPERTIES BUNDLE TRUE)
else ()
	add_library(
	    ${PROJECT_NAME} SHARED
	    ${RPC_LIBRARY_SOURCE_FILES}
	    ${MAVLINK_LIBRARY_SOURCE_FILES}
	    ${AIRLIB_LIBRARY_SOURCE_FILES}
	    ${AIRSIMWRAPPER_LIBRARY_SOURCE_FILES}
    )
endif ()

target_link_libraries(${PROJECT_NAME} ${CMAKE_THREAD_LIBS_INIT}  -lstdc++ -lpthread ${Boost_LIBRARIES})



##################### Build Options #############################3
# Rpc
RpcCheckMSVN()
RpcCmakePkg()
RpcMSVNConfig()
