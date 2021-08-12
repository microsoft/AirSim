# generated from ament_cmake_export_libraries/cmake/template/ament_cmake_export_libraries.cmake.in

set(_exported_libraries "airsim_interfaces__rosidl_generator_c;airsim_interfaces__rosidl_typesupport_c;airsim_interfaces__rosidl_typesupport_cpp")
set(_exported_library_names "")

# populate airsim_interfaces_LIBRARIES
if(NOT _exported_libraries STREQUAL "")
  # loop over libraries, either target names or absolute paths
  list(LENGTH _exported_libraries _length)
  set(_i 0)
  while(_i LESS _length)
    list(GET _exported_libraries ${_i} _arg)

    # pass linker flags along
    if("${_arg}" MATCHES "^-" AND NOT "${_arg}" MATCHES "^-[l|framework]")
      list(APPEND airsim_interfaces_LIBRARIES "${_arg}")
      math(EXPR _i "${_i} + 1")
      continue()
    endif()

    if("${_arg}" MATCHES "^(debug|optimized|general)$")
      # remember build configuration keyword
      # and get following library
      set(_cfg "${_arg}")
      math(EXPR _i "${_i} + 1")
      if(_i EQUAL _length)
        message(FATAL_ERROR "Package 'airsim_interfaces' passes the build configuration keyword '${_cfg}' as the last exported library")
      endif()
      list(GET _exported_libraries ${_i} _library)
    else()
      # the value is a library without a build configuration keyword
      set(_cfg "")
      set(_library "${_arg}")
    endif()
    math(EXPR _i "${_i} + 1")

    if(NOT IS_ABSOLUTE "${_library}")
      # search for library target relative to this CMake file
      set(_lib "NOTFOUND")
      find_library(
        _lib NAMES "${_library}"
        PATHS "${airsim_interfaces_DIR}/../../../lib"
        NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH
      )

      if(NOT _lib)
        # warn about not existing library and ignore it
        message(FATAL_ERROR "Package 'airsim_interfaces' exports the library '${_library}' which couldn't be found")
      elseif(NOT IS_ABSOLUTE "${_lib}")
        # the found library must be an absolute path
        message(FATAL_ERROR "Package 'airsim_interfaces' found the library '${_library}' at '${_lib}' which is not an absolute path")
      elseif(NOT EXISTS "${_lib}")
        # the found library must exist
        message(FATAL_ERROR "Package 'airsim_interfaces' found the library '${_lib}' which doesn't exist")
      else()
        list(APPEND airsim_interfaces_LIBRARIES ${_cfg} "${_lib}")
      endif()

    else()
      if(NOT EXISTS "${_library}")
        # the found library must exist
        message(WARNING "Package 'airsim_interfaces' exports the library '${_library}' which doesn't exist")
      else()
        list(APPEND airsim_interfaces_LIBRARIES ${_cfg} "${_library}")
      endif()
    endif()
  endwhile()
endif()

# find_library() library names with optional LIBRARY_DIRS
# and add the libraries to airsim_interfaces_LIBRARIES
if(NOT _exported_library_names STREQUAL "")
  # loop over library names
  # but remember related build configuration keyword if available
  list(LENGTH _exported_library_names _length)
  set(_i 0)
  while(_i LESS _length)
    list(GET _exported_library_names ${_i} _arg)
    # pass linker flags along
    if("${_arg}" MATCHES "^-" AND NOT "${_arg}" MATCHES "^-[l|framework]")
      list(APPEND airsim_interfaces_LIBRARIES "${_arg}")
      math(EXPR _i "${_i} + 1")
      continue()
    endif()

    if("${_arg}" MATCHES "^(debug|optimized|general)$")
      # remember build configuration keyword
      # and get following library name
      set(_cfg "${_arg}")
      math(EXPR _i "${_i} + 1")
      if(_i EQUAL _length)
        message(FATAL_ERROR "Package 'airsim_interfaces' passes the build configuration keyword '${_cfg}' as the last exported target")
      endif()
      list(GET _exported_library_names ${_i} _library)
    else()
      # the value is a library target without a build configuration keyword
      set(_cfg "")
      set(_library "${_arg}")
    endif()
    math(EXPR _i "${_i} + 1")

    # extract optional LIBRARY_DIRS from library name
    string(REPLACE ":" ";" _library_dirs "${_library}")
    list(GET _library_dirs 0 _library_name)
    list(REMOVE_AT _library_dirs 0)

    set(_lib "NOTFOUND")
    if(NOT _library_dirs)
      # search for library in the common locations
      find_library(
        _lib
        NAMES "${_library_name}"
      )
      if(NOT _lib)
        # warn about not existing library and later ignore it
        message(WARNING "Package 'airsim_interfaces' exports library '${_library_name}' which couldn't be found")
      endif()
    else()
      # search for library in the specified directories
      find_library(
        _lib
        NAMES "${_library_name}"
        PATHS ${_library_dirs}
        NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH
      )
      if(NOT _lib)
        # warn about not existing library and later ignore it
        message(WARNING "Package 'airsim_interfaces' exports library '${_library_name}' with LIBRARY_DIRS '${_library_dirs}' which couldn't be found")
      endif()
    endif()
    if(_lib)
      list(APPEND airsim_interfaces_LIBRARIES ${_cfg} "${_lib}")
    endif()
  endwhile()
endif()

# TODO(dirk-thomas) deduplicate airsim_interfaces_LIBRARIES
# while maintaining library order
# as well as build configuration keywords
# as well as linker flags
