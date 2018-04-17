cmake_minimum_required(VERSION 3.5.0)
project(rpclib_wrapper)

IF(UNIX)
    set(RPC_LIB_DEFINES " -D MSGPACK_PP_VARIADICS_MSVC=0")
    set(CMAKE_CXX_FLAGS "${RPC_LIB_DEFINES} ${CMAKE_CXX_FLAGS}")    
ENDIF()

add_subdirectory("${AIRSIM_ROOT}/external/rpclib/${RPCLIB_VERSION_FOLDER}" "${CMAKE_CURRENT_BINARY_DIR}/rpclib")