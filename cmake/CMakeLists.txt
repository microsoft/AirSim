cmake_minimum_required(VERSION 3.5.0)
project(AirSim)
set(SOLUTION_NAME ${PROJECT_NAME})

set(AIRSIM_VERSION_MAJOR 1)
set(AIRSIM_VERSION_MIDDLE 5)
set(AIRSIM_VERSION_MINOR 0)
set(AIRSIM_VERSION ${AIRSIM_VERSION_MAJOR}.${AIRSIM_VERSION_MIDDLE}.${AIRSIM_VERSION_MINOR})

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake-modules")
list(APPEND CMAKE_PREFIX_PATH "${CMAKE_INSTALL_PREFIX}")

option(BUILD_EXAMPLES "build examples" OFF)
option(BUILD_TESTS "build tests" OFF)

include(CommonSetup)
CommonSetup()

add_subdirectory("3rdparty")

# libraries
add_subdirectory("MavLinkCom")
add_subdirectory("AirLib")

# tests
if (BUILD_TESTS)
    add_subdirectory("AirLibUnitTests")
endif()

# examples
if(BUILD_EXAMPLES)
    add_subdirectory("HelloDrone")
    add_subdirectory("HelloSpawnedDrones")
    add_subdirectory("HelloCar")
    add_subdirectory("DroneShell")
    add_subdirectory("DroneServer")
endif()

include(GNUInstallDirs)
install(EXPORT ${SOLUTION_NAME}-export
  FILE
    ${SOLUTION_NAME}Targets.cmake
  NAMESPACE
    ${SOLUTION_NAME}::
  DESTINATION
    ${CMAKE_INSTALL_DATADIR}/${SOLUTION_NAME}
)

include(CMakePackageConfigHelpers)
configure_package_config_file(
    "${PROJECT_SOURCE_DIR}/cmake-modules/${SOLUTION_NAME}Config.cmake.in"
    "${PROJECT_BINARY_DIR}/${SOLUTION_NAME}Config.cmake"
    INSTALL_DESTINATION ${CMAKE_INSTALL_DATADIR}/${SOLUTION_NAME}
)

write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/${SOLUTION_NAME}ConfigVersion.cmake
  VERSION ${AIRSIM_VERSION}
  COMPATIBILITY SameMajorVersion)

install(FILES "${CMAKE_CURRENT_BINARY_DIR}/${SOLUTION_NAME}ConfigVersion.cmake"
              "${CMAKE_CURRENT_BINARY_DIR}/${SOLUTION_NAME}Config.cmake"
        DESTINATION ${CMAKE_INSTALL_DATADIR}/${SOLUTION_NAME})