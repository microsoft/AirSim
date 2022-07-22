cmake_minimum_required(VERSION 3.11)
set(SOLUTION_NAME AirsimWrapper)

set(AIRSIM_WRAPPER_VERSION_MAJOR 1)
set(AIRSIM_WRAPPER_VERSION_MIDDLE 5)
set(AIRSIM_WRAPPER_VERSION_MINOR 0)
set(AIRSIM_WRAPPER_VERSION ${AIRSIM_VERSION_MAJOR}.${AIRSIM_VERSION_MIDDLE}.${AIRSIM_VERSION_MINOR})

if (${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    set(MACOSX TRUE)
endif()

LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

INCLUDE("${CMAKE_CURRENT_SOURCE_DIR}/cmake/airsimwrapper-setup.cmake")

project(AirsimWrapper VERSION 0)

BuildAirsimWrapper()

###################### Link source files to library ######################################
if (${APPLE})
	add_library(${PROJECT_NAME} MODULE ${AIRSIMWRAPPER_LIBRARY_SOURCE_FILES})
	set_target_properties(${PROJECT_NAME} PROPERTIES BUNDLE TRUE)
else()
	add_library(${PROJECT_NAME} SHARED ${AIRSIMWRAPPER_LIBRARY_SOURCE_FILES})
endif()

if (WIN32)
  # TODO: it is better to use GenerateExportHeader instead of this
  # WINDOWS_EXPORT_ALL_SYMBOLS may lead to fail if count of symbols will exceed 65536
	set_target_properties(${PROJECT_NAME} PROPERTIES 
		WINDOWS_EXPORT_ALL_SYMBOLS ON
	)
endif()

# do not replace this before add_library and add_executable it will break vcpkg integration
find_path(AIRSIM_ROOT NAMES AirSim.sln PATHS ".." "../.." "../../.." "../../../.." "../../../../.." "../../../../../..")
message(AirSim Root directory: ${AIRSIM_ROOT})
add_subdirectory(${AIRSIM_ROOT}/cmake Airsim)
target_link_libraries(${PROJECT_NAME} PRIVATE AirSim::AirLib)

find_package(Threads REQUIRED)
target_link_libraries(${PROJECT_NAME} PRIVATE Threads::Threads)

if (UNIX)
	target_link_libraries(${PROJECT_NAME} PRIVATE -lstdc++)
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
    "${PROJECT_SOURCE_DIR}/cmake/${SOLUTION_NAME}Config.cmake.in"
    "${PROJECT_BINARY_DIR}/${SOLUTION_NAME}Config.cmake"
    INSTALL_DESTINATION ${CMAKE_INSTALL_DATADIR}/${SOLUTION_NAME}
)

write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/${SOLUTION_NAME}ConfigVersion.cmake
  VERSION ${AIRSIM_WRAPPER_VERSION}
  COMPATIBILITY SameMajorVersion)

install(FILES "${CMAKE_CURRENT_BINARY_DIR}/${SOLUTION_NAME}ConfigVersion.cmake"
              "${CMAKE_CURRENT_BINARY_DIR}/${SOLUTION_NAME}Config.cmake"
        DESTINATION ${CMAKE_INSTALL_DATADIR}/${SOLUTION_NAME})

install(TARGETS ${PROJECT_NAME}
	EXPORT ${SOLUTION_NAME}-export
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)