# ------------------------------------------------------------------------------
# Project: Aetherion
# Copyright (c) 2025, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: MIT
# License-Filename: LICENSE
# ------------------------------------------------------------------------------

# Findlibode.cmake
#
# Usage:
#   find_package(libode REQUIRED)
#
# Provides:
#   libode_FOUND
#   libode_LIBRARIES  -> libode::libode
#   libode::libode    -> header-only INTERFACE target

include(FindPackageHandleStandardArgs)

# Prefer value from Dependencies.cmake
if(DEFINED LIBODE_INCLUDE_DIR)
    set(_libode_include "${LIBODE_INCLUDE_DIR}")
elseif(DEFINED libode_SOURCE_DIR)
    set(_libode_include "${libode_SOURCE_DIR}")
endif()

find_package_handle_standard_args(
    libode
    REQUIRED_VARS _libode_include
)

if(libode_FOUND AND NOT TARGET libode::libode)
    add_library(libode::libode INTERFACE IMPORTED)
set_target_properties(libode::libode PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES
     "${libode_SOURCE_DIR}/include" 
    )
endif()

if(libode_FOUND)
    set(libode_LIBRARIES libode::libode)
endif()
