# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_rosmaster_a1_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED rosmaster_a1_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(rosmaster_a1_FOUND FALSE)
  elseif(NOT rosmaster_a1_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(rosmaster_a1_FOUND FALSE)
  endif()
  return()
endif()
set(_rosmaster_a1_CONFIG_INCLUDED TRUE)

# output package information
if(NOT rosmaster_a1_FIND_QUIETLY)
  message(STATUS "Found rosmaster_a1: 0.1.0 (${rosmaster_a1_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'rosmaster_a1' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT rosmaster_a1_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(rosmaster_a1_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${rosmaster_a1_DIR}/${_extra}")
endforeach()
