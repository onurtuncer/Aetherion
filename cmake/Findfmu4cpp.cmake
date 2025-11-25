# ------------------------------------------------------------------------------
# Project: Aetherion
# Copyright (c) 2025, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: MIT
# License-Filename: LICENSE
# ------------------------------------------------------------------------------

# Findfmu4cpp.cmake
#
# Lightweight Find-module that cooperates with FetchContent.
# Usage:
#   find_package(fmu4cpp REQUIRED)
# Exposes:
#   fmu4cpp_FOUND
#   fmu4cpp_LIBRARIES  -> fmu4cpp::fmu4cpp

include(FindPackageHandleStandardArgs)

set(fmu4cpp_TARGET "")

# Case 1: Dependencies.cmake already did FetchContent_MakeAvailable(fmu4cpp)
if(TARGET fmu4cpp::fmu4cpp)
    set(fmu4cpp_TARGET fmu4cpp::fmu4cpp)
elseif(TARGET fmu4cpp)
    # Provide a namespaced alias if only a bare target exists
    add_library(fmu4cpp::fmu4cpp ALIAS fmu4cpp)
    set(fmu4cpp_TARGET fmu4cpp::fmu4cpp)
endif()

find_package_handle_standard_args(
    fmu4cpp
    REQUIRED_VARS fmu4cpp_TARGET
)

if(fmu4cpp_FOUND)
    set(fmu4cpp_LIBRARIES "${fmu4cpp_TARGET}")
endif()
