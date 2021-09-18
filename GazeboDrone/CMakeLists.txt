cmake_minimum_required(VERSION 3.5.0)
project(GazeboDrone)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(gazebo REQUIRED)

list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_executable(${PROJECT_NAME} src/main.cpp)
target_include_directories(${PROJECT_NAME} PRIVATE ${GAZEBO_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PRIVATE ${GAZEBO_LIBRARIES})

add_subdirectory(../cmake AirSim)
target_link_libraries(${PROJECT_NAME} PRIVATE AirSim::AirLib)