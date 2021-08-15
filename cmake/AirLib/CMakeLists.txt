cmake_minimum_required(VERSION 3.5.0)
project(AirLib)

include(GNUInstallDirs)

LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/../cmake-modules") 
INCLUDE("${CMAKE_CURRENT_LIST_DIR}/../cmake-modules/CommonSetup.cmake")
CommonSetup()

file(GLOB_RECURSE ${PROJECT_NAME}_sources 
  ${AIRSIM_ROOT}/${PROJECT_NAME}/src/api/*.cpp
  ${AIRSIM_ROOT}/${PROJECT_NAME}/src/common/common_utils/*.cpp
  ${AIRSIM_ROOT}/${PROJECT_NAME}/src/safety/*.cpp
  ${AIRSIM_ROOT}/${PROJECT_NAME}/src/vehicles/car/api/*.cpp
  ${AIRSIM_ROOT}/${PROJECT_NAME}/src/vehicles/multirotor/*.cpp
  ${AIRSIM_ROOT}/${PROJECT_NAME}/src/vehicles/multirotor/api/*.cpp
)

add_library(${PROJECT_NAME} STATIC ${${PROJECT_NAME}_sources})
add_library(${SOLUTION_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

target_include_directories(${PROJECT_NAME} 
  PUBLIC
    $<BUILD_INTERFACE:${AIRSIM_ROOT}/AirLib/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/${SOLUTION_NAME}/${PROJECT_NAME}>
  PRIVATE
    $<BUILD_INTERFACE:${AIRSIM_ROOT}/AirLib>
)

set_target_properties(${PROJECT_NAME} 
  PROPERTIES
    CXX_EXTENSIONS OFF
)

find_package(rpclib CONFIG REQUIRED)
target_link_libraries(${PROJECT_NAME} PUBLIC rpclib::rpc)

find_package(Eigen3 CONFIG REQUIRED)
target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen)

target_link_libraries(${PROJECT_NAME} PUBLIC MavLinkCom)

install(DIRECTORY "${AIRSIM_ROOT}/${PROJECT_NAME}/include/" DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/${SOLUTION_NAME}/${PROJECT_NAME}")
install(TARGETS ${PROJECT_NAME}
	EXPORT ${SOLUTION_NAME}-export
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

#string(SUBSTRING ${CMAKE_STATIC_LINKER_FLAGS} 9 -1 "BUILD_PLATFORM")
#find_package(Threads REQUIRED)
#target_link_libraries(${PROJECT_NAME} ${CMAKE_THREAD_LIBS_INIT})
#target_link_libraries(${PROJECT_NAME} AirLib)
#target_link_libraries(${PROJECT_NAME} MavLinkCom)
#target_link_libraries(${PROJECT_NAME} ../AirLib/deps/rpclib/lib/${BUILD_PLATFORM}/$(Configuration)/rpc)
