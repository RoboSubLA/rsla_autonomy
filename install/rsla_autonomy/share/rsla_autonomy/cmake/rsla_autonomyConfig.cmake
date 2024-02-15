# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_rsla_autonomy_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED rsla_autonomy_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(rsla_autonomy_FOUND FALSE)
  elseif(NOT rsla_autonomy_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(rsla_autonomy_FOUND FALSE)
  endif()
  return()
endif()
set(_rsla_autonomy_CONFIG_INCLUDED TRUE)

# output package information
if(NOT rsla_autonomy_FIND_QUIETLY)
  message(STATUS "Found rsla_autonomy: 0.0.0 (${rsla_autonomy_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'rsla_autonomy' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT rsla_autonomy_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(rsla_autonomy_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${rsla_autonomy_DIR}/${_extra}")
endforeach()
