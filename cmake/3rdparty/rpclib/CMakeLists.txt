cmake_minimum_required(VERSION 3.11)
project(Eigen_3rdparty)

include(../cmake/3rdparty_utils.cmake)

download_3rdparty(
    NAME rpclib
    GIT_REPOSITORY https://github.com/rpclib/rpclib
    GIT_TAG v2.3.0
    SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/download
)

if (UNIX)
    set(CONFIG_ARGS -DCMAKE_POSITION_INDEPENDENT_CODE=ON)
endif()

configure_3rdparty(
    SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/download
    BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/build
    OPTIONS ${CONFIG_ARGS}
)

install_3rdparty(
    BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/build
)

