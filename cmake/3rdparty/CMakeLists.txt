cmake_minimum_required(VERSION 3.5.0)
project(Airsim_3rdparty)

if(WIN32) # need to find vcvarsall.bat due to shell environment doesn't pass to execute_process and ExternalProject_Add
    if(NOT VSWHERE_PATH)
        set(DEFAULT_VSWHERE_PATH "$ENV{ProgramFiles\(x86\)}/Microsoft Visual Studio/Installer/vswhere.exe")
        if (EXISTS ${DEFAULT_VSWHERE_PATH})
            set(VSWHERE_PATH ${DEFAULT_VSWHERE_PATH} CACHE PATH "path to vswhere.exe (default: ${DEFAULT_VSWHERE_PATH})")
        endif()
        if (NOT VSWHERE_PATH)
            message(FATAL_ERROR "Vswhere not found, try to install a Visual Studio or specify a directory containing vswhere.exe by VSWHERE_PATH cache variable")
        endif()
        execute_process(COMMAND ${VSWHERE_PATH} -latest -property installationPath OUTPUT_VARIABLE VS_INSTALLATION_PATH)
        string(REPLACE "\\" "/" VS_INSTALLATION_PATH ${VS_INSTALLATION_PATH})
        string(REPLACE "\n" "" VS_INSTALLATION_PATH ${VS_INSTALLATION_PATH})
        set(VSDEVCMD ${VS_INSTALLATION_PATH}/Common7/Tools/VsDevCmd.bat)
        message(DEBUG "${VSDEVCMD}")
        if (NOT EXISTS "${VSDEVCMD}")
            message(FATAL_ERROR "vsdevcmd.bat not found, try to install c++ tools for Visual Studio")
        endif()
        
        # x64 supposed
        # We need batch file to fix error "The input line is too long."
        file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/cmake.bat "
            setlocal
            start cmd /c \"${VSDEVCMD}\" -no_logo -arch=amd64 && \"${CMAKE_COMMAND}\" %*
            endlocal
        ")
        set(CMAKE_COMMAND "${CMAKE_CURRENT_BINARY_DIR}/cmake.bat")
    endif()
else()
    set(CMAKE_COMMAND_EXTERNAL ${CMAKE_COMMAND})
endif()


set(INSTALL_DIR ${CMAKE_INSTALL_PREFIX})
set(CMAKE_ARGS 
    "-DCMAKE_GENERATOR=${CMAKE_GENERATOR}"
    "-DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}"
    "-DCMAKE_GENERATOR=${CMAKE_GENERATOR}"
    "-DCMAKE_C_COMPILER=${CMAKE_C_COMPILER}"
    "-DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}"
    "-DCMAKE_TOOLCHAIN_FILE=${CMAKE_TOOLCHAIN_FILE}"
    "-DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}"
)
set(3RD_PARTIES rpclib Eigen3)
foreach(3RD_PARTY IN LISTS 3RD_PARTIES)
    find_package(${3RD_PARTY} QUIET)
    if(FORCE_INSTALL_3RDPARTY)
        unset(${3RD_PARTY}_FOUND)
    endif()
    if (NOT ${3RD_PARTY}_FOUND) # suppose we have already installed dependency and we don't need to install it
        message(STATUS "${3RD_PARTY} was not found, it will be installed by cmake")
        set(SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/.download/${3RD_PARTY})
        set(BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/.build/${3RD_PARTY})
        include(${3RD_PARTY}/variables.cmake)
        configure_file(${CMAKE_CURRENT_LIST_DIR}/CMakeLists.txt.in ${CMAKE_CURRENT_BINARY_DIR}/.external/${3RD_PARTY}/CMakeLists.txt @ONLY)
        execute_process(
            COMMAND ${CMAKE_COMMAND} 
                -S${CMAKE_CURRENT_BINARY_DIR}/.external/${3RD_PARTY}
                -B${CMAKE_CURRENT_BINARY_DIR}/.external/${3RD_PARTY}/.build
                ${CMAKE_ARGS})
        execute_process(COMMAND ${CMAKE_COMMAND} --build ${CMAKE_CURRENT_BINARY_DIR}/.external/${3RD_PARTY}/.build)
        execute_process(COMMAND ${CMAKE_COMMAND} --install ${CMAKE_CURRENT_BINARY_DIR}/.external/${3RD_PARTY}/.build)
    endif()
endforeach()