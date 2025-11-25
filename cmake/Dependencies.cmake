# ------------------------------------------------------------------------------
# Project: Aetherion
# Copyright (c) 2025, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: MIT
# License-Filename: LICENSE
# ------------------------------------------------------------------------------

# Guard against multiple inclusion
if(TARGET fmu4cpp::fmu4cpp)
    return()
endif()

include(FetchContent)

FetchContent_Declare(
    fmu4cpp
    GIT_REPOSITORY https://github.com/Ecos-platform/fmu4cpp.git
    GIT_TAG        master
)

FetchContent_MakeAvailable(fmu4cpp)

FetchContent_Declare(
    libode
    GIT_REPOSITORY https://github.com/markmbaum/libode.git
    GIT_TAG        2d8ac5b
)

FetchContent_GetProperties(libode)
if(NOT libode_POPULATED)
    FetchContent_Populate(libode)

    # Header-only: just expose include dir (ode/ode_rk_4.h lives here)
    set(LIBODE_INCLUDE_DIR "${libode_SOURCE_DIR}" CACHE PATH "libode include dir")
endif()

#set(ECOS_BUILD_CLI OFF)     # Set to ON for building ecos command-line-interface
#set(ECOS_BUILD_CLIB OFF)    # Set to ON for building C API
#set(ECOS_WITH_PROXYFMU OFF) # Set to ON for remoting
#
#FetchContent_Declare(
#        ecos
#        GIT_REPOSITORY https://github.com/Ecos-platform/ecos.git
#        GIT_TAG master
#)
#
#FetchContent_MakeAvailable(ecos)



# ==============================================================================
# CppAD - header-only, NO subbuild, we just want include/ and a target
# ==============================================================================

FetchContent_Declare(
    cppad
    GIT_REPOSITORY https://github.com/coin-or/CppAD.git
    GIT_TAG        master          # pin if you like
    GIT_SHALLOW    TRUE
)

FetchContent_GetProperties(cppad)
if(NOT cppad_POPULATED)
    FetchContent_Populate(cppad)  # <-- JUST downloads; no SOURCE_SUBDIR
    # Create an INTERFACE target with the proper include dir
    if(NOT TARGET CppAD::cppad)
        add_library(CppAD::cppad INTERFACE IMPORTED)
        target_include_directories(CppAD::cppad INTERFACE
            "${cppad_SOURCE_DIR}/include"
        )
    endif()
endif()


# ==============================================================================
# Eigen - header-only, same pattern
# ==============================================================================

FetchContent_Declare(
    eigen
    GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
    GIT_TAG        3.4.0           # pick a tag you like
    GIT_SHALLOW    TRUE
)

FetchContent_GetProperties(eigen)
if(NOT eigen_POPULATED)
    FetchContent_Populate(eigen)
    if(NOT TARGET Eigen::Eigen)
        add_library(Eigen::Eigen INTERFACE IMPORTED)
        target_include_directories(Eigen::Eigen INTERFACE
            "${eigen_SOURCE_DIR}"
        )
    endif()
endif()