cmake_minimum_required(VERSION 3.5.0)
project(AirSim)

# Try to create a compilation database, which is useful to have when working
# with clang tooling
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(EXTERNAL_DIR "${CMAKE_SOURCE_DIR}/external" CACHE DOCUMENTATION "Directory to install external dependencies" )
set(AIRSIM_ROOT "${CMAKE_SOURCE_DIR}" )

LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake-modules") 
INCLUDE("${CMAKE_CURRENT_LIST_DIR}/cmake-modules/CommonSetup.cmake")
CommonSetup()

# Turn on ability to create folders to organize projects
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

# Set C++ version
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(THREADS_PREFER_PTHREAD_FLAG ON)
set(CMAKE_THREAD_PREFER_PTHREAD ON)
find_package(Threads REQUIRED)

if(MSVC)
  # Set Visual Studio-specific options
  add_definitions(-DUNICODE)
  add_compile_options(/MP)
  add_compile_options(/bigobj)
  add_compile_options(/W4)
  if(STRICT_MODE)
    add_compile_options(/WX)
  endif()
endif()

add_subdirectory("rpclib_wrapper")
add_subdirectory("AirLib")
add_subdirectory("MavLinkCom")
add_subdirectory("AirLibUnitTests")
add_subdirectory("HelloDrone")
add_subdirectory("HelloCar")
add_subdirectory("DroneShell")
add_subdirectory("DroneServer")


