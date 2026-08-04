# ------------------------------------------------------------------------------
# Project: Aetherion Copyright (c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: MIT License-Filename: LICENSE
# ------------------------------------------------------------------------------

if(BUILD_DOCS)
  return()
endif()

# ------------------------------------------------------------------------------
# CppAD -- provided by vcpkg on every platform (see vcpkg.json and cmake/VcpkgToolchain.cmake), or by any prefix on
# CMAKE_PREFIX_PATH when the build is configured with -DAETHERION_USE_VCPKG=OFF. cmake/FindCppAD.cmake defines the
# CppAD::cppad imported target; CppAD ships no CMake package configuration of its own.
# ------------------------------------------------------------------------------
find_package(CppAD MODULE REQUIRED)
message(STATUS "Using CppAD ${CppAD_VERSION}: ${CppAD_LIBRARY}")

# ------------------------------------------------------------------------------
# Eigen3 (vendored headers)
# ------------------------------------------------------------------------------
set(EIGEN3_VENDOR_DIR "${PROJECT_SOURCE_DIR}/vendor/eigen")

if(EXISTS "${EIGEN3_VENDOR_DIR}/Eigen/Dense")
  message(STATUS "Using vendored Eigen headers in ${EIGEN3_VENDOR_DIR}")

  # IMPORTED GLOBAL (matching CppAD::cppad above): a plain, non-imported target here would need to be added to
  # AetherionTargets' export set and would fail install(EXPORT) with "requires target eigen3_vendor that is not in any
  # export set", since Aetherion links it PUBLIC. IMPORTED GLOBAL targets are exempt from that requirement.
  add_library(
    eigen3_vendor
    INTERFACE
    IMPORTED
    GLOBAL)
  target_include_directories(eigen3_vendor INTERFACE "${EIGEN3_VENDOR_DIR}")

  add_library(Eigen3::Eigen ALIAS eigen3_vendor)
else()
  message(FATAL_ERROR "Eigen3 not found: vendored headers at " "${EIGEN3_VENDOR_DIR}/Eigen/Dense")
endif()

# ------------------------------------------------------------------------------
# nlohmann/json (vendored header-only)
# ------------------------------------------------------------------------------
add_library(nlohmann_json INTERFACE)

target_include_directories(nlohmann_json INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/vendor)

target_compile_definitions(nlohmann_json INTERFACE NLOHMANN_JSON_HEADER_ONLY)

# ------------------------------------------------------------------------------
# ecos (vendored submodule) -- FMI co-simulation engine, used for FMU smoke tests
# ------------------------------------------------------------------------------
set(ECOS_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(ECOS_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(ECOS_BUILD_CLI OFF CACHE BOOL "" FORCE)
set(ECOS_BUILD_CLIB OFF CACHE BOOL "" FORCE)
set(ECOS_WITH_PROXYFMU OFF CACHE BOOL "" FORCE)
add_subdirectory("${PROJECT_SOURCE_DIR}/vendor/ecos" EXCLUDE_FROM_ALL)
