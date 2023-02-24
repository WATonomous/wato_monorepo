# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_transformer_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED transformer_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(transformer_FOUND FALSE)
  elseif(NOT transformer_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(transformer_FOUND FALSE)
  endif()
  return()
endif()
set(_transformer_CONFIG_INCLUDED TRUE)

# output package information
if(NOT transformer_FIND_QUIETLY)
  message(STATUS "Found transformer: 0.0.0 (${transformer_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'transformer' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${transformer_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(transformer_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${transformer_DIR}/${_extra}")
endforeach()
