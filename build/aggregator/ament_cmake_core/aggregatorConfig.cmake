# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_aggregator_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED aggregator_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(aggregator_FOUND FALSE)
  elseif(NOT aggregator_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(aggregator_FOUND FALSE)
  endif()
  return()
endif()
set(_aggregator_CONFIG_INCLUDED TRUE)

# output package information
if(NOT aggregator_FIND_QUIETLY)
  message(STATUS "Found aggregator: 0.0.0 (${aggregator_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'aggregator' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${aggregator_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(aggregator_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${aggregator_DIR}/${_extra}")
endforeach()
