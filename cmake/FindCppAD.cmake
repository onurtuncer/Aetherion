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
# Exposes:
#   CppAD_FOUND
#   CppAD_LIBRARIES           -> CppAD::cppad
#   CppAD_INCLUDE_DIRS (opt.) -> include path if we create the target

include(FindPackageHandleStandardArgs)

set(CppAD_TARGET "")

# Case 1: Dependencies.cmake already added CppAD with an imported target
if(TARGET CppAD::cppad)
    set(CppAD_TARGET CppAD::cppad)

# Case 2: Header-only fallback using FetchContent variables
elseif(DEFINED cppad_SOURCE_DIR)
    # CppAD headers live under ${cppad_SOURCE_DIR}/include/cppad
    set(CPPAD_INCLUDE_DIR "${cppad_SOURCE_DIR}/include")

    add_library(CppAD::cppad INTERFACE IMPORTED)
    set_target_properties(CppAD::cppad PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${CPPAD_INCLUDE_DIR}"
    )

    set(CppAD_TARGET CppAD::cppad)
    set(CppAD_INCLUDE_DIRS "${CPPAD_INCLUDE_DIR}")
endif()

find_package_handle_standard_args(
    CppAD
    REQUIRED_VARS CppAD_TARGET
)

if(CppAD_FOUND)
    set(CppAD_LIBRARIES "${CppAD_TARGET}")
endif()
