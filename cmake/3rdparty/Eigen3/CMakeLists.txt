cmake_minimum_required(VERSION 3.11)
project(Eigen_3rdparty)

include(../cmake/3rdparty_utils.cmake)

download_3rdparty(
    NAME eigen3
    GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
    GIT_TAG 3.3.9
    SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/download
)

configure_3rdparty(
    SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/download
    BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/build
    OPTIONS 
        -DBUILD_TESTING=OFF
)

install_3rdparty(
    BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/build
)

