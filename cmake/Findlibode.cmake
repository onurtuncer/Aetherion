# ------------------------------------------------------------------------------
# Project: Aetherion
# Copyright (c) 2025, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: MIT
# License-Filename: LICENSE
# ------------------------------------------------------------------------------

# Findode.cmake
#
# Usage:
#   find_package(ode REQUIRED)
# Exposes:
#   ode_FOUND
#   ode_LIBRARIES -> ode::ode

include(FindPackageHandleStandardArgs)

set(ode_TARGET "")

# Case 1: Brought in via FetchContent (Dependencies.cmake)
if(TARGET ode::ode)
    set(ode_TARGET ode::ode)
elseif(TARGET ode)
    # Create the canonical namespaced target if only "ode" exists
    add_library(ode::ode ALIAS ode)
    set(ode_TARGET ode::ode)
endif()

find_package_handle_standard_args(
    ode
    REQUIRED_VARS ode_TARGET
)

if(ode_FOUND)
    set(ode_LIBRARIES "${ode_TARGET}")
endif()
