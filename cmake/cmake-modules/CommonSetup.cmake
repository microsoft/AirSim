# Common setup instructions shared by all AirSim CMakeLists.

macro(CommonTargetLink)
    target_link_libraries(${PROJECT_NAME} ${CMAKE_THREAD_LIBS_INIT})
    #target_link_libraries(c++abi)
endmacro(CommonTargetLink)

macro(IncludeEigen)
    include_directories(${AIRSIM_ROOT}/AirLib/deps/eigen3)
endmacro(IncludeEigen)

macro(AddExecutableSource)
    set(PROJECT_CPP ${PROJECT_NAME}_sources)
    file(GLOB_RECURSE PROJECT_CPP "${AIRSIM_ROOT}/${PROJECT_NAME}/*.cpp")
    add_executable(${PROJECT_NAME} ${PROJECT_CPP})
endmacro(AddExecutableSource)

macro(SetupConsoleBuild)
    IF(UNIX)
    ELSE()
        set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /D_CONSOLE ")
        set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /SUBSYSTEM:CONSOLE")
    ENDIF()
endmacro(SetupConsoleBuild)

macro(CommonSetup)
    message(STATUS "Running CommonSetup...")

    find_package(Threads REQUIRED)

    find_path(AIRSIM_ROOT NAMES AirSim.sln PATHS ".." "../.." "../../.." "../../../.." "../../../../.." "../../../../../..")
    message(STATUS "found AIRSIM_ROOT=${AIRSIM_ROOT}")

    #setup output paths
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/output/lib)
    SET(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/output/bin)
    SET(LIBRARY_OUTPUT_PATH ${CMAKE_LIBRARY_OUTPUT_DIRECTORY})

    #setup include and lib for rpclib which will be referenced by other projects
    set(RPCLIB_VERSION_FOLDER rpclib-2.2.1)
    set(RPC_LIB_INCLUDES " ${AIRSIM_ROOT}/external/rpclib/${RPCLIB_VERSION_FOLDER}/include")
    #name of .a file with lib prefix
    set(RPC_LIB rpc)

    #what is our build type debug or release?
    string( TOLOWER "${CMAKE_BUILD_TYPE}" BUILD_TYPE)

    IF(UNIX)
        set(RPC_LIB_DEFINES "-D MSGPACK_PP_VARIADICS_MSVC=0")
        set(BUILD_TYPE "linux")

        if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "AppleClang")
            #TODO: need to check why below is needed
            set(CMAKE_CXX_STANDARD 14)
            set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D__CLANG__")
        else ()
            set(CMAKE_CXX_FLAGS "\
                -std=c++14 -stdlib=libc++ -ggdb -Wall -Wextra -Wstrict-aliasing -Wunreachable-code -Wcast-qual -Wctor-dtor-privacy \
                -Wdisabled-optimization -Wformat=2 -Winit-self -Wmissing-include-dirs -Wswitch-default \
                -Wold-style-cast -Woverloaded-virtual -Wredundant-decls -Wshadow -Wstrict-overflow=5 -Wswitch-default -Wundef \
                -Wno-variadic-macros -Wno-parentheses -Wno-unused-function -Wno-unused -Wno-documentation -fdiagnostics-show-option \
                -pthread \
                ${RPC_LIB_DEFINES} ${CMAKE_CXX_FLAGS}")

            if (${CMAKE_CXX_COMPILER_ID} MATCHES "Clang")
            else()
                set(CXX_EXP_LIB "-lstdc++fs -fmax-errors=10 -Wnoexcept -Wstrict-null-sentinel")
            endif ()
        endif ()

        set(BUILD_PLATFORM "x64")
        set(CMAKE_POSITION_INDEPENDENT_CODE ON)

    ELSE()
        #windows cmake build is experimental
        set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D_WIN32_WINNT=0x0600 /GS /W4 /wd4100 /wd4505 /wd4820 /wd4464 /wd4514 /wd4710 /wd4571 /Zc:wchar_t /ZI /Zc:inline /fp:precise /D_SCL_SECURE_NO_WARNINGS /D_CRT_SECURE_NO_WARNINGS /D_UNICODE /DUNICODE /WX- /Zc:forScope /Gd /EHsc ")
        set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /NXCOMPAT /DYNAMICBASE /INCREMENTAL:NO ")

        if("${BUILD_TYPE}" STREQUAL "debug")
          set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /D_DEBUG /MDd /RTC1 /Gm /Od ")
        elseif("${BUILD_TYPE}" STREQUAL "release")
          set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MD /O2 /Oi /GL /Gm- /Gy /TP ")
        else()
          message(FATAL_ERROR "Please specify '-D CMAKE_BUILD_TYPE=Debug' or Release on the cmake command line")
        endif()
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

