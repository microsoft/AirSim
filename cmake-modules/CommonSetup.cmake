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

    #setup output paths
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/output/lib)
    SET(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/output/bin)
    SET(LIBRARY_OUTPUT_PATH ${CMAKE_LIBRARY_OUTPUT_DIRECTORY})

    #libcxx which we will use with specific version of clang
    SET(LIBCXX_INC_PATH ${AIRSIM_ROOT}/llvm-build/output/include/c++/v1)
    SET(LIBCXX_LIB_PATH ${AIRSIM_ROOT}/llvm-build/output/lib)

    #setup include and lib for rpclib which will be referenced by other projects
    set(RPCLIB_VERSION_FOLDER rpclib-2.2.1)
    set(RPC_LIB_INCLUDES " ${AIRSIM_ROOT}/external/rpclib/${RPCLIB_VERSION_FOLDER}/include")
    #name of .a file with lib prefix
    set(RPC_LIB rpc)

    IF(UNIX)
        set(RPC_LIB_DEFINES "-D MSGPACK_PP_VARIADICS_MSVC=0")

        if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "AppleClang")
            #TODO: need to check why below is needed
            set(CMAKE_CXX_STANDARD 14)
            set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D__CLANG__")
        else ()
            # other flags used in Unreal: -funwind-tables  -fdiagnostics-format=msvc -fno-inline  -Werror -fno-omit-frame-pointer  -fstack-protector -O2
            # TODO: add back -Wunused-parameter -Wno-documentation after rpclib can be compiled
            set(CMAKE_CXX_FLAGS "\
                -std=c++14 -ggdb -Wall -Wextra -Wstrict-aliasing -Wunreachable-code -Wcast-qual -Wctor-dtor-privacy \
                -Wdisabled-optimization -Wformat=2 -Winit-self -Wmissing-include-dirs -Wswitch-default \
                -Wold-style-cast -Woverloaded-virtual -Wredundant-decls -Wshadow -Wstrict-overflow=5 -Wswitch-default -Wundef \
                -Wno-variadic-macros -Wno-parentheses -Wno-unused-function -Wno-unused -Wno-documentation -fdiagnostics-show-option \
                -pthread \
                ${RPC_LIB_DEFINES} ${CMAKE_CXX_FLAGS}")

            if (${CMAKE_CXX_COMPILER_ID} MATCHES "Clang")
                # make sure to match the compiler flags with which the Unreal
                # Engine is built with
                set(CMAKE_CXX_FLAGS "\
                    -nostdinc++ -ferror-limit=10 -isystem ${LIBCXX_INC_PATH} \
                    -D__CLANG__ ${CMAKE_CXX_FLAGS}")

                # removed -lsupc++ from below (Git issue # 678)
                set(CMAKE_EXE_LINKER_FLAGS "\
                    ${CMAKE_EXE_LINKER_FLAGS} -stdlib=libc++ -lc++ -lc++abi -lm -lc \
                    -L ${LIBCXX_LIB_PATH} -rpath ${LIBCXX_LIB_PATH}")

                #do not use experimental as it might potentially cause ABI issues
                #set(CXX_EXP_LIB "-lc++experimental")

                # set same options that Unreal sets in debug builds
                set (CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} -funwind-tables -fdiagnostics-format=msvc -fno-inline -fno-omit-frame-pointer -fstack-protector")

            else()
                set(CXX_EXP_LIB "-lstdc++fs -fmax-errors=10 -Wnoexcept -Wstrict-null-sentinel")
            endif ()
        endif ()

        set(BUILD_PLATFORM "x64")
        set(CMAKE_POSITION_INDEPENDENT_CODE ON)

    ELSE()
        #windows cmake build is experimental
        set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D_WIN32_WINNT=0x0600 /GS /W4 /wd4100 /wd4505 /wd4820 /wd4464 /wd4514 /wd4710 /wd4571 /Zc:wchar_t /Zc:inline /D_SCL_SECURE_NO_WARNINGS /D_CRT_SECURE_NO_WARNINGS /D_UNICODE /DUNICODE /Zc:forScope /Gd /EHsc ")
        set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /NXCOMPAT /DYNAMICBASE /INCREMENTAL:NO ")

        set (CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} /D_DEBUG /MDd /RTC1 /ZI /Gm /Od ")
        set (CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} /MD /O2 /Oi /GL /Gm- /Gy /TP ")
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

macro(GetLocalSources varname curdir)
    file(GLOB children RELATIVE ${PROJECT_SOURCE_DIR}/${curdir} ${PROJECT_SOURCE_DIR}/${curdir}/*)    
    foreach(child ${children})
        if(IS_DIRECTORY ${PROJECT_SOURCE_DIR}/${curdir}/${child})
            GetLocalSources(${varname} ${curdir}/${child})
        else()
            string(REPLACE "/" "\\" groupname ${curdir})
            if ("${groupname}" STREQUAL ".")
                set(groupname "src")
            endif()
            source_group(${groupname} FILES ${PROJECT_SOURCE_DIR}/${curdir}/${child})
            get_filename_component(FILEEXT ${child} EXT)
            if ("${FILEEXT}" STREQUAL ".h" OR "${FILEEXT}" STREQUAL ".hpp" OR "${FILEEXT}" STREQUAL ".cpp")
                LIST(APPEND ${varname} ${PROJECT_SOURCE_DIR}/${curdir}/${child})
            endif()
        endif()
    endforeach()
endmacro()
