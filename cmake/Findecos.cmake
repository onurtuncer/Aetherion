# ------------------------------------------------------------------------------
# Project: Aetherion
# Copyright (c) 2025, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: MIT
# License-Filename: LICENSE
# ------------------------------------------------------------------------------

# Findexos.cmake
#
# Usage:
#   find_package(ecos REQUIRED)
# Exposes:
#   ecos_FOUND
#   ecos_LIBRARIES -> libecos

include(FindPackageHandleStandardArgs)

set(ecos_TARGET "")

# Case 1: Dependencies.cmake did FetchContent_MakeAvailable(ecos)
# Ecos README shows linking with "libecos"
if(TARGET libecos)
    set(ecos_TARGET libecos)
endif()

find_package_handle_standard_args(
    ecos
    REQUIRED_VARS ecos_TARGET
)

if(ecos_FOUND)
    set(ecos_LIBRARIES "${ecos_TARGET}")
endif()
