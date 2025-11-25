# ------------------------------------------------------------------------------
# Project: Aetherion
# Copyright (c) 2025, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: MIT
# License-Filename: LICENSE
# ------------------------------------------------------------------------------

# Findfmu4cpp.cmake
#
# Usage:
#   find_package(fmu4cpp REQUIRED)
#
# Assumes fmu4cpp is brought in via FetchContent in Dependencies.cmake:
#
#   FetchContent_Declare(
#       fmu4cpp
#       GIT_REPOSITORY https://github.com/Ecos-platform/fmu4cpp.git
#       GIT_TAG        master
#   )
#   FetchContent_MakeAvailable(fmu4cpp)
#
# Provides:
#   FMU4CPP_FOUND
#   FMU4CPP_INCLUDE_DIRS
#   fmu4cpp::fmu4cpp   (INTERFACE imported target)

include(FindPackageHandleStandardArgs)

# 1) Try to use the source dir from FetchContent
#    (Dependencies.cmake should have called FetchContent_MakeAvailable(fmu4cpp))
if(NOT DEFINED fmu4cpp_SOURCE_DIR)
    # Optional: allow user override via cache
    set(fmu4cpp_ROOT "" CACHE PATH "Root of fmu4cpp source (if not using FetchContent)")
    if(fmu4cpp_ROOT)
        set(fmu4cpp_SOURCE_DIR "${fmu4cpp_ROOT}")
    endif()
endif()

# 2) If we still don't know where it is, report not found
if(NOT DEFINED fmu4cpp_SOURCE_DIR)
    find_package_handle_standard_args(
        fmu4cpp
        REQUIRED_VARS fmu4cpp_SOURCE_DIR
    )
    return()
endif()

# 3) Include directories
#    README shows: #include <fmu4cpp/fmu_base.hpp>
#    We don't know if that lives under root or export/, so we add both.
set(FMU4CPP_INCLUDE_DIRS
    "${fmu4cpp_SOURCE_DIR}"
    "${fmu4cpp_SOURCE_DIR}/export"
)

# 4) Create an imported interface target if it doesn't exist yet
if(NOT TARGET fmu4cpp::fmu4cpp)
    add_library(fmu4cpp::fmu4cpp INTERFACE IMPORTED)
    set_target_properties(fmu4cpp::fmu4cpp PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${FMU4CPP_INCLUDE_DIRS}"
    )
endif()

# 5) Mark as found
find_package_handle_standard_args(
    fmu4cpp
    REQUIRED_VARS FMU4CPP_INCLUDE_DIRS
)

if(FMU4CPP_FOUND)
    # Optional compatibility variable
    set(FMU4CPP_LIBRARIES fmu4cpp::fmu4cpp)
endif()

