# Common setup instructions shared by all AirSim CMakeLists.

macro(CommonSetup)

    find_package(Threads REQUIRED)

    message(STATUS "Running CommonSetup...")   
    
    find_path(AIRSIM_ROOT NAMES AirSim.sln PATHS ".." "../.." "../../.." "../../../.." "../../../../.." "../../../../../..")
    message(STATUS "found AIRSIM_ROOT=${AIRSIM_ROOT}")

    SET(LIBRARY_OUTPUT_PATH ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}) 
	SET(EXECUTABLE_OUTPUT_PATH ${AIRSIM_ROOT}/cmake/output/bin) 

    IF(UNIX)
        ## I had to remove the following for Eigen to build properly: -Wlogical-op -Wsign-promo 
        set(AIRLIB_OVERRIDES " -Wno-error=unused-parameter") 
        ## Mavlink requires turning off -pedantic  and -Wno-error=switch-default 
        set(MAVLINK_OVERRIDES "-Wno-error=switch-default ") 
        set(RPC_LIB_DEFINES "-D MSGPACK_PP_VARIADICS_MSVC=0")
        if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "AppleClang")
            set(CMAKE_CXX_STANDARD 14)
            set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D__CLANG__")
        else ()
            ##TODO: Werror removed temporarily. It should be added back after Linux build is stable
            # other flags used in Unreal: -funwind-tables  -fdiagnostics-format=msvc -fno-inline  -fno-omit-frame-pointer  -fstack-protector -O2
            set(CMAKE_CXX_FLAGS "-std=c++14 -ggdb  -Wall -Wextra -Wstrict-aliasing -Wunreachable-code -Wcast-qual -Wctor-dtor-privacy -Wdisabled-optimization -Wformat=2 -Winit-self -Wmissing-include-dirs -Wold-style-cast -Woverloaded-virtual -Wredundant-decls -Wshadow -Wstrict-overflow=5 -Wswitch-default -Wundef -Wno-unused -Wno-variadic-macros -Wno-parentheses -fdiagnostics-show-option ${MAVLINK_OVERRIDES} ${AIRLIB_OVERRIDES} ${RPC_LIB_DEFINES} ${CMAKE_CXX_FLAGS}")

            if (${CMAKE_CXX_COMPILER_ID} MATCHES "Clang")
                # make sure to match the compiler flags with which the Unreal
                # Engine is built with
                set(CMAKE_CXX_FLAGS "-stdlib=libc++ -D__CLANG__ ${CMAKE_CXX_FLAGS}")
                set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -stdlib=libc++ -lc++ -lc++abi ")
                set(CXX_EXP_LIB "-lc++experimental")
            else()
                set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -lpthread -ldl -Wnoexcept -Wstrict-null-sentinel -fmax-errors=10")
                set(CXX_EXP_LIB "-lstdc++fs")
            endif ()
        endif ()
        set(BUILD_PLATFORM "x64")
        set(CMAKE_POSITION_INDEPENDENT_CODE ON)

    ELSE()
        set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D_WIN32_WINNT=0x0600 /GS /W4 /wd4100 /wd4505 /wd4820 /wd4464 /wd4514 /wd4710 /wd4571 /Zc:wchar_t /ZI /Zc:inline /fp:precise /D_SCL_SECURE_NO_WARNINGS /D_CRT_SECURE_NO_WARNINGS /D_UNICODE /DUNICODE /WX- /Zc:forScope /Gd /EHsc ")
        set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /NXCOMPAT /DYNAMICBASE /INCREMENTAL:NO ")
    ENDIF()

    ## common boost settings to make sure we are all on the same page
    set(Boost_USE_STATIC_LIBS ON) 
    set(Boost_USE_MULTITHREADED ON)  
    #set(Boost_USE_STATIC_RUNTIME ON)  

    ## strip /machine: from CMAKE_STATIC_LINKER_FLAGS
    if(NOT "${CMAKE_STATIC_LINKER_FLAGS}" STREQUAL "")
      string(SUBSTRING ${CMAKE_STATIC_LINKER_FLAGS} 9 -1 "BUILD_PLATFORM")
    endif() 
    
    IF(UNIX)
        set(BUILD_TYPE "linux")

    ELSE()
        string( TOLOWER "${CMAKE_BUILD_TYPE}" BUILD_TYPE)
        if("${BUILD_TYPE}" STREQUAL "debug")
          set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /D_DEBUG /MDd /RTC1 /Gm /Od ")
        elseif("${BUILD_TYPE}" STREQUAL "release")
          set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MD /O2 /Oi /GL /Gm- /Gy /TP ")
        else()
          message(FATAL_ERROR "Please specify '-D CMAKE_BUILD_TYPE=Debug' or Release on the cmake command line")
        endif()
    endif() 
	
	set(RPC_LIB_INCLUDES " ${AIRSIM_ROOT}/external/rpclib/include") 
	set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/output/lib)
	set(RPC_LIB ${CMAKE_PROJECT_NAME}-${RPCLIB_NAME_SUFFIX})
	set(RPC_BIN_NAME "${AIRSIM_ROOT}/cmake/${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/lib${RPC_LIB}.a")
endmacro(CommonSetup)

