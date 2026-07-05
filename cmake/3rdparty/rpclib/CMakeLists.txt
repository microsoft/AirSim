cmake_minimum_required(VERSION 3.11)
project(Eigen_3rdparty)

include(../cmake/3rdparty_utils.cmake)
include(../../cmake-modules/CommonSetup.cmake)

CommonSetup()

download_3rdparty(
    NAME rpclib
    GIT_REPOSITORY https://github.com/rpclib/rpclib
    GIT_TAG v2.3.0
    SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/download
)

if(UNIX)
    set(RPC_LIB_DEFINES " -D MSGPACK_PP_VARIADICS_MSVC=0")
    set(CROSS_PLATFORM_ARGS
        "-DCMAKE_CXX_FLAGS=${RPC_LIB_DEFINES} ${CMAKE_CXX_FLAGS}" 
        "-DCMAKE_POSITION_INDEPENDENT_CODE=ON"
    )
endif()

configure_3rdparty(
    SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/download
    BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/build
    OPTIONS ${CROSS_PLATFORM_ARGS}
)

install_3rdparty(
    BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/build
)