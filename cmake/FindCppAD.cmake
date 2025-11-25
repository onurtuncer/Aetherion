# ------------------------------------------------------------------------------
# Project: Aetherion
# Copyright (c) 2025, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: MIT
# License-Filename: LICENSE
# ------------------------------------------------------------------------------

# FindCppAD.cmake
#
# Usage:
#   find_package(CppAD REQUIRED)
#
# Provides:
#   CppAD_FOUND
#   CppAD_INCLUDE_DIRS
#   CppAD::cppad  (INTERFACE imported target)

include(FindPackageHandleStandardArgs)

set(CPPAD_INCLUDE_DIRS "")

# Prefer the include dir we set in Dependencies.cmake
if(NOT CPPAD_INCLUDE_DIRS AND DEFINED CPPAD_INCLUDE_DIR)
    set(CPPAD_INCLUDE_DIRS "${CPPAD_INCLUDE_DIR}")
elseif(NOT CPPAD_INCLUDE_DIRS AND DEFINED cppad_SOURCE_DIR)
    set(CPPAD_INCLUDE_DIRS "${cppad_SOURCE_DIR}/include")
endif()

find_package_handle_standard_args(
    CppAD
    REQUIRED_VARS CPPAD_INCLUDE_DIRS
)

if(CppAD_FOUND AND NOT TARGET CppAD::cppad)
    add_library(CppAD::cppad INTERFACE IMPORTED)
    set_target_properties(CppAD::cppad PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${CPPAD_INCLUDE_DIRS}"
    )
endif()
