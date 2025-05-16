# Common setup instructions shared by all AirSim CMakeLists.

macro(AddExecutableSource)
    set(PROJECT_CPP ${PROJECT_NAME}_sources)
    file(GLOB_RECURSE PROJECT_CPP "${AIRSIM_ROOT}/${PROJECT_NAME}/*.cpp")
    add_executable(${PROJECT_NAME} ${PROJECT_CPP})
endmacro(AddExecutableSource)

macro(CommonSetup)
    find_package(Threads REQUIRED)
    find_path(AIRSIM_ROOT NAMES AirSim.sln PATHS ".." "../.." "../../.." "../../../.." "../../../../.." "../../../../../.." REQUIRED)

    #setup output paths
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/output/lib)
    SET(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/output/bin)
    SET(LIBRARY_OUTPUT_PATH ${CMAKE_LIBRARY_OUTPUT_DIRECTORY})

    IF(UNIX)
        set(RPC_LIB_DEFINES "-D MSGPACK_PP_VARIADICS_MSVC=0")
        set(BUILD_TYPE "linux")
        set(CMAKE_CXX_STANDARD 17)

        if (APPLE)
            set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wstrict-aliasing -D__CLANG__")
        else ()
            set(CMAKE_CXX_FLAGS "\
                -Wall -Wextra \
                -Wnon-virtual-dtor -Woverloaded-virtual \
                -Wno-variadic-macros -Wno-unused-function -Wno-unused \
                -pthread \
                ${RPC_LIB_DEFINES} ${CMAKE_CXX_FLAGS}")

            if (${CMAKE_CXX_COMPILER_ID} MATCHES "Clang")
                set(CMAKE_CXX_FLAGS "-stdlib=libc++ -Wno-documentation -Wno-unknown-warning-option ${CMAKE_CXX_FLAGS}")
                find_package(LLVM REQUIRED CONFIG)
                set(CXX_EXP_LIB "-L${LLVM_LIBRARY_DIRS} -lc++fs -ferror-limit=10")
            else()
                set(CXX_EXP_LIB "-lstdc++fs -fmax-errors=10 -Wnoexcept -Wstrict-null-sentinel")
            endif ()
        endif ()

        set(BUILD_PLATFORM "x64")
        set(CMAKE_POSITION_INDEPENDENT_CODE ON)
        if (CMAKE_BUILD_TYPE MATCHES Release)
            set(CMAKE_CXX_FLAGS "-O3 ${CMAKE_CXX_FLAGS}")
        endif ()
    ENDIF()

    ## TODO: we are not using Boost any more so below shouldn't be needed
    ## common boost settings to make sure we are all on the same page
    set(Boost_USE_STATIC_LIBS ON)
    set(Boost_USE_MULTITHREADED ON)
    #set(Boost_USE_STATIC_RUNTIME ON)

    ## TODO: probably should set x64 explicitly
    ## strip x64 from /machine:x64 from CMAKE_STATIC_LINKER_FLAGS and set in BUILD_PLATFORM
    if(NOT "${CMAKE_STATIC_LINKER_FLAGS}" STREQUAL "")
      string(SUBSTRING ${CMAKE_STATIC_LINKER_FLAGS} 9 -1 "BUILD_PLATFORM")
    endif()

endmacro(CommonSetup)
