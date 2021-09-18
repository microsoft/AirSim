macro(init_env)
    if(WIN32) # need to find vsdevcmd.bat
        find_file(VSDEVCMD VsDevCmd.bat)
        if(NOT VSDEVCMD AND NOT VSWHERE_PATH)
            set(DEFAULT_VSWHERE_PATH "$ENV{ProgramFiles\(x86\)}/Microsoft Visual Studio/Installer/vswhere.exe")
            if (EXISTS ${DEFAULT_VSWHERE_PATH})
                set(VSWHERE_PATH ${DEFAULT_VSWHERE_PATH} CACHE PATH "path to vswhere.exe (default: ${DEFAULT_VSWHERE_PATH})")
            endif()
        endif()
        if (NOT VSDEVCMD AND VSWHERE_PATH)
            execute_process(COMMAND ${VSWHERE_PATH} -latest -property installationPath OUTPUT_VARIABLE VS_INSTALLATION_PATH)
            string(REPLACE "\\" "/" VS_INSTALLATION_PATH ${VS_INSTALLATION_PATH})
            string(REPLACE "\n" "" VS_INSTALLATION_PATH ${VS_INSTALLATION_PATH})
            set(VSDEVCMD ${VS_INSTALLATION_PATH}/Common7/Tools/VsDevCmd.bat)
        endif()
        message(DEBUG "${VSDEVCMD}")
        if (NOT EXISTS "${VSDEVCMD}")
            message(FATAL_ERROR "vsdevcmd.bat not found, try to install c++ tools for Visual Studio")
        endif()
        
        # x64 supposed
        # We need batch file to fix error "The input line is too long."
        file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/cmake.bat "
            setlocal
            \"${VSDEVCMD}\" -no_logo -arch=amd64 && \"${CMAKE_COMMAND}\" %*
            endlocal
        ")
        set(CMAKE_COMMAND "${CMAKE_CURRENT_BINARY_DIR}/cmake.bat")
    endif()
endmacro()

function(download_3rdparty)
    set(options )
    set(oneValueArgs NAME GIT_REPOSITORY GIT_TAG SOURCE_DIR)
    set(multiValueArgs )
    cmake_parse_arguments(DOWNLOAD_3RDPARTY "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    include(FetchContent)
    FetchContent_Declare(
        ${DOWNLOAD_3RDPARTY_NAME}_3rdparty
        GIT_REPOSITORY ${DOWNLOAD_3RDPARTY_GIT_REPOSITORY}
        GIT_TAG        ${DOWNLOAD_3RDPARTY_GIT_TAG}
        SOURCE_DIR ${DOWNLOAD_3RDPARTY_SOURCE_DIR}
    )
    FetchContent_Populate(${DOWNLOAD_3RDPARTY_NAME}_3rdparty)
endfunction()

function(configure_3rdparty)
    set(options )
    set(oneValueArgs SOURCE_DIR BINARY_DIR)
    set(multiValueArgs OPTIONS)
    cmake_parse_arguments(CONFIGURE_3RDPARTY "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    init_env()
    set(CMAKE_ARGS
        -DCMAKE_GENERATOR=${CMAKE_GENERATOR}
        -DCMAKE_GENERATOR_PLATFORM=${CMAKE_GENERATOR_PLATFORM}
        -DCMAKE_C_COMPILER=${CMAKE_C_COMPILER}
        -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}
        -DCMAKE_TOOLCHAIN_FILE=${CMAKE_TOOLCHAIN_FILE}
        -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
        ${CONFIGURE_3RDPARTY_OPTIONS}
    )
    execute_process(COMMAND ${CMAKE_COMMAND} 
        -S${CONFIGURE_3RDPARTY_SOURCE_DIR}
        -B${CONFIGURE_3RDPARTY_BINARY_DIR}
        ${CMAKE_ARGS})
endfunction()


function(install_3rdparty)
    set(options )
    set(oneValueArgs BINARY_DIR)
    set(multiValueArgs )
    cmake_parse_arguments(INSTALL_3RDPARTY "${options}" "${oneValueArgs}" "multiValueArgs" ${ARGN})

    init_env()
    execute_process(COMMAND ${CMAKE_COMMAND} --build ${INSTALL_3RDPARTY_BINARY_DIR} --config ${CMAKE_BUILD_TYPE})
    execute_process(COMMAND ${CMAKE_COMMAND} --install ${INSTALL_3RDPARTY_BINARY_DIR} --config ${CMAKE_BUILD_TYPE} --prefix "${CMAKE_INSTALL_PREFIX}")
endfunction()