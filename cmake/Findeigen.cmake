# ------------------------------------------------------------------------------
# Project: Aetherion
# Copyright (c) 2025, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: MIT
# License-Filename: LICENSE
# ------------------------------------------------------------------------------

# Findeigen.cmake
#
# Minimal Eigen find module that cooperates with FetchContent.
#
# Usage:
#   find_package(eigen REQUIRED)
#
# Provides:
#   EIGEN_FOUND
#   EIGEN_INCLUDE_DIRS
#   Eigen::Eigen (INTERFACE imported target)

include(FindPackageHandleStandardArgs)

set(_eigen_target "")
set(EIGEN_INCLUDE_DIRS "")

# Case 1: Some other part of the world already defined an Eigen target
if(TARGET Eigen3::Eigen)
    set(_eigen_target Eigen3::Eigen)

    # Try to extract includes if possible
    get_target_property(_eigen_includes Eigen3::Eigen INTERFACE_INCLUDE_DIRECTORIES)
    if(_eigen_includes)
        set(EIGEN_INCLUDE_DIRS "${_eigen_includes}")
    endif()

elseif(TARGET Eigen::Eigen)
    set(_eigen_target Eigen::Eigen)

    get_target_property(_eigen_includes Eigen::Eigen INTERFACE_INCLUDE_DIRECTORIES)
    if(_eigen_includes)
        set(EIGEN_INCLUDE_DIRS "${_eigen_includes}")
    endif()
endif()

# Case 2: Fetched via FetchContent in Dependencies.cmake
if(NOT _eigen_target)
    # FetchContent_MakeAvailable(eigen) should have set this
    if(DEFINED eigen_SOURCE_DIR)
        # Eigen’s layout: <eigen_SOURCE_DIR>/Eigen/...
        set(EIGEN_INCLUDE_DIRS "${eigen_SOURCE_DIR}")

        add_library(Eigen::Eigen INTERFACE IMPORTED)
        set_target_properties(Eigen::Eigen PROPERTIES
            INTERFACE_INCLUDE_DIRECTORIES "${EIGEN_INCLUDE_DIRS}"
        )

        set(_eigen_target Eigen::Eigen)
    endif()
endif()

find_package_handle_standard_args(
    eigen
    REQUIRED_VARS _eigen_target
)

if(EIGEN_FOUND)
    # Optional convenience alias: some code might expect this variable
    set(EIGEN_LIBRARIES Eigen::Eigen)
endif()
