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

  # IMPORTED GLOBAL (matching CppAD::cppad above): a plain, non-imported target would have to join AetherionTargets'
  # export set. It carries the name Eigen3::Eigen itself rather than being an ALIAS of an internal name: the export
  # records the real name in Aetherion's INTERFACE_LINK_LIBRARIES, and an internal name (eigen3_vendor, before) is
  # unknown to an installed SDK's consumers, whose AetherionConfig.cmake defines Eigen3::Eigen (pre-DOI audit F07).
  add_library(
    Eigen3::Eigen
    INTERFACE
    IMPORTED
    GLOBAL)
  target_include_directories(Eigen3::Eigen INTERFACE "${EIGEN3_VENDOR_DIR}")
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
# ecos (vendored submodule) -- FMI co-simulation engine, used only by the FMU smoke test under
# tests/ThirdPartyIntegration. It is added only when the tests are built, so the library, examples and FMUs configure
# from a source archive that has no submodules (a `git archive` or a GitHub source tarball -- pre-DOI audit finding
# R01). With tests on, a missing submodule is a clear error instead of CMake's "does not contain a CMakeLists.txt".
# ------------------------------------------------------------------------------
if(AETHERION_BUILD_TESTS)
  if(NOT EXISTS "${PROJECT_SOURCE_DIR}/vendor/ecos/CMakeLists.txt")
    message(
      FATAL_ERROR
        "vendor/ecos is empty: the ecos submodule is needed by the test suite. Run
" "  git submodule update --init vendor/ecos
" "or configure with -DAETHERION_BUILD_TESTS=OFF to build without tests.")
  endif()
  set(ECOS_BUILD_TESTS OFF CACHE BOOL "" FORCE)
  set(ECOS_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
  set(ECOS_BUILD_CLI OFF CACHE BOOL "" FORCE)
  set(ECOS_BUILD_CLIB OFF CACHE BOOL "" FORCE)
  set(ECOS_WITH_PROXYFMU OFF CACHE BOOL "" FORCE)
  add_subdirectory("${PROJECT_SOURCE_DIR}/vendor/ecos" EXCLUDE_FROM_ALL)
endif()
