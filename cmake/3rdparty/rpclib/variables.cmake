set(3RDPARTY_OPTIONS \"${CMAKE_ARGS}\")
if (UNIX)
    list(APPEND 3RDPARTY_OPTIONS "-DCMAKE_POSITION_INDEPENDENT_CODE=ON")
endif()

set(GIT_REPOSITORY https://github.com/rpclib/rpclib)
set(GIT_TAG v2.3.0)