# ------------------------------------------------------------------------------
# Project: Aetherion
# Copyright (c) 2025, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: MIT
# License-Filename: LICENSE
# ------------------------------------------------------------------------------

cmake_minimum_required(VERSION 3.26)

project(Aetherion VERSION 1.0 LANGUAGES CXX)


# ---- Hack for CppAD on Windows: it *requires* PkgConfig just to probe Eigen ----
if (WIN32)
    # Point PKG_CONFIG_EXECUTABLE at cmake.exe as a dummy.
    # FindPkgConfig only checks that the program exists; CppAD's Eigen
    # support is optional, so failed pkg-config calls will just mean
    # "no Eigen", which is fine for us.
    set(PKG_CONFIG_EXECUTABLE "${CMAKE_COMMAND}" CACHE FILEPATH "Dummy pkg-config for CppAD" FORCE)

    # Be explicit that we don't want Eigen support in CppAD anyway
    set(include_eigen OFF CACHE BOOL "Disable Eigen support in CppAD" FORCE)
endif()

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

# === CppAD via add_subdirectory ===
set(CPPAD_TESTS OFF CACHE BOOL "Disable CppAD tests")
set(CPPAD_EXAMPLES OFF CACHE BOOL "Disable CppAD examples")
set(CPPAD_DOXYGEN OFF CACHE BOOL "Disable CppAD docs")
set(CPPAD_IPOPT OFF CACHE BOOL "Disable Ipopt solver")
set(CPPAD_PKGCONFIG OFF CACHE BOOL "Disable pkgconfig")
set(CPPAD_BOOSTVECTOR OFF CACHE BOOL "Disable boost vector")
set(CPPAD_EIGENVECTOR ON CACHE BOOL "Enable Eigen vector (optional)")
set(CPPAD_CXX_STANDARD 20 CACHE STRING "Use C++20")

add_subdirectory(vendor/cppad)


include(Dependencies)


find_package(fmu4cpp REQUIRED)
find_package(libode REQUIRED)

add_executable(aetherion
    src/main.cpp
    # add other .cpp files here
)



target_link_libraries(aetherion
    PRIVATE
        fmu4cpp::fmu4cpp
        libode::libode         # header-only, INTERFACE
)

if (BUILD_DOCS)
  add_subdirectory(doc)
endif()

if (ENABLE_STATIC_ANALYSIS)
  include(cppcheck)
endif()

if(AETHERION_BUILD_TESTS)
	add_subdirectory(tests)
endif()

## Only include tests if enabled
#if(AETHERION_BUILD_TESTS)
#    include(CTest)        # Defines BUILD_TESTING, adds test target
#    add_subdirectory(tests)
#endif()