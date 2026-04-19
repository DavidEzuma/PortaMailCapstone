# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_portamail_coordinator_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED portamail_coordinator_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(portamail_coordinator_FOUND FALSE)
  elseif(NOT portamail_coordinator_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(portamail_coordinator_FOUND FALSE)
  endif()
  return()
endif()
set(_portamail_coordinator_CONFIG_INCLUDED TRUE)

# output package information
if(NOT portamail_coordinator_FIND_QUIETLY)
  message(STATUS "Found portamail_coordinator: 0.1.0 (${portamail_coordinator_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'portamail_coordinator' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT portamail_coordinator_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(portamail_coordinator_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${portamail_coordinator_DIR}/${_extra}")
endforeach()
