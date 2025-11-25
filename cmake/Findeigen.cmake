# ------------------------------------------------------------------------------
# Project: Aetherion
# Copyright (c) 2025, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: MIT
# License-Filename: LICENSE
# ------------------------------------------------------------------------------

# Findeigen.cmake

include(FindPackageHandleStandardArgs)

if(DEFINED EIGEN_INCLUDE_DIR)
    set(EIGEN_INCLUDE_DIRS "${EIGEN_INCLUDE_DIR}")
elseif(DEFINED eigen_SOURCE_DIR)
    set(EIGEN_INCLUDE_DIRS "${eigen_SOURCE_DIR}")
endif()

find_package_handle_standard_args(
    eigen
    REQUIRED_VARS EIGEN_INCLUDE_DIRS
)

if(eigen_FOUND AND NOT TARGET Eigen::Eigen)
    add_library(Eigen::Eigen INTERFACE IMPORTED)
    set_target_properties(Eigen::Eigen PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${EIGEN_INCLUDE_DIRS}"
    )
endif()

