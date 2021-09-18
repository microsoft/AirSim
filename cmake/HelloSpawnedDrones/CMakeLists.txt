cmake_minimum_required(VERSION 3.5.0)
project(HelloSpawnedDrones)

LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/../cmake-modules") 
INCLUDE("${CMAKE_CURRENT_LIST_DIR}/../cmake-modules/CommonSetup.cmake")
CommonSetup()

SetupConsoleBuild()

AddExecutableSource()
			
target_include_directories(${PROJECT_NAME} PRIVATE
  $<BUILD_INTERFACE:${AIRSIM_ROOT}/HelloSpawnedDrones>
)

target_link_libraries(${PROJECT_NAME} PRIVATE AirLib)

install(TARGETS ${PROJECT_NAME}
	EXPORT ${SOLUTION_NAME}-export
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)