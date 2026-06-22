# ------------------------------------------------------------------------------
# Project: Aetherion Copyright (c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: MIT License-Filename: LICENSE
# ------------------------------------------------------------------------------

cmake_minimum_required(VERSION 3.26)

file(STRINGS "${CMAKE_CURRENT_SOURCE_DIR}/version.txt" projectVersion)

project(
  Aetherion
  VERSION ${projectVersion}
  DESCRIPTION "Flight dynamics simulation library with FMI/FMU support"
  LANGUAGES CXX)

# ------------------------------------------------------------------------------
# CMake module path� -- must come before any include() that uses it
# ------------------------------------------------------------------------------
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

# ------------------------------------------------------------------------------
# Options � all option() declarations live here, parsed before anything else
# ------------------------------------------------------------------------------
include(cmake/AetherionOptions.cmake)

# ------------------------------------------------------------------------------
# Global compiler settings
# ------------------------------------------------------------------------------
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(AETHERION_WARNINGS_AS_ERRORS)
  if(MSVC)
    add_compile_options(/WX)
  else()
    add_compile_options(-Werror)
  endif()
endif()

if(AETHERION_ENABLE_COVERAGE)
  if(MSVC)
    message(WARNING "[Aetherion] Coverage instrumentation is not supported with MSVC.")
  else()
    add_compile_options(--coverage)
    add_link_options(--coverage)
  endif()
endif()

# ------------------------------------------------------------------------------
# Docs-only build � configure only the doc target and exit early
# ------------------------------------------------------------------------------
if(BUILD_DOCS)
  add_subdirectory(doc)
  return()
endif()

include("cmake/generate_fmu.cmake")
include(GNUInstallDirs)

# ------------------------------------------------------------------------------
# Vendored dependencies
# ------------------------------------------------------------------------------
# --- fmu4cpp vendor integration ---
set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
set(FMU4CPP_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(FMU4CPP_TESTS OFF CACHE BOOL "" FORCE)
add_subdirectory(vendor/fmu4cpp/export EXCLUDE_FROM_ALL)

# ------------------------------------------------------------------------------
# Other vendored dependencies (Eigen, nlohmann_json, CppAD, ecos, �)
# ------------------------------------------------------------------------------
include(Dependencies)

# ------------------------------------------------------------------------------
# Library + FMU targets
# ------------------------------------------------------------------------------
add_subdirectory(src)

# ------------------------------------------------------------------------------
# Static analysis
# ------------------------------------------------------------------------------
if(ENABLE_STATIC_ANALYSIS)
  include(cppcheck)
endif()

# ------------------------------------------------------------------------------
# Tests
# ------------------------------------------------------------------------------
if(AETHERION_BUILD_TESTS)
  include(CTest)
  enable_testing()
  add_subdirectory(tests)
endif()

# ------------------------------------------------------------------------------
# Examples
# ------------------------------------------------------------------------------
if(AETHERION_BUILD_EXAMPLES)
  add_subdirectory(src/Examples)
endif()

# ------------------------------------------------------------------------------
# Install rules (optional, guarded by AETHERION_INSTALL=ON)
# ------------------------------------------------------------------------------
if(AETHERION_INSTALL)
  include(CMakePackageConfigHelpers)

  # fmu4cpp OBJECT libraries cannot be installed/exported directly. Wrap them in thin static libraries that can be
  # exported as targets.
  add_library(aetherion_fmu4cpp_base STATIC)
  target_sources(aetherion_fmu4cpp_base PRIVATE $<TARGET_OBJECTS:fmu4cpp_base>)
  target_include_directories(
    aetherion_fmu4cpp_base PUBLIC $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/vendor/fmu4cpp/export/include>
                                  $<INSTALL_INTERFACE:include>)
  set_target_properties(aetherion_fmu4cpp_base PROPERTIES EXPORT_NAME fmu4cpp_base)

  add_library(aetherion_fmu4cpp_fmi2 STATIC)
  target_sources(aetherion_fmu4cpp_fmi2 PRIVATE $<TARGET_OBJECTS:fmu4cpp_fmi2>)
  target_compile_definitions(aetherion_fmu4cpp_fmi2 PUBLIC FMI2)
  target_include_directories(
    aetherion_fmu4cpp_fmi2 PUBLIC $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/vendor/fmu4cpp/export/include>
                                  $<INSTALL_INTERFACE:include>)
  set_target_properties(aetherion_fmu4cpp_fmi2 PROPERTIES EXPORT_NAME fmu4cpp_fmi2)

  # ── Library targets ──────────────────────────────────────────────────────────
  install(
    TARGETS Aetherion aetherion_fmu4cpp_base aetherion_fmu4cpp_fmi2
    EXPORT AetherionTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

  # descriptionGenerator tool (needed by generateFMU() at SWIL build time)
  install(TARGETS descriptionGenerator RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

  # ── Headers ────────────────────────────────────────────────────────────────── Aetherion public headers →
  # include/Aetherion/
  install(
    DIRECTORY Aetherion/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/Aetherion
    FILES_MATCHING
    PATTERN "*.h"
    PATTERN "*.hpp")

  # fmu4cpp public headers → include/fmu4cpp/ and include/fmi2/ etc.
  install(
    DIRECTORY vendor/fmu4cpp/export/include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    FILES_MATCHING
    PATTERN "*.h"
    PATTERN "*.hpp")

  # Eigen3 headers → include/Eigen/
  install(DIRECTORY vendor/eigen/Eigen DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

  # CppAD headers
  if(WIN32)
    install(
      DIRECTORY vendor/cppad/x64-Release/include/
      DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
      FILES_MATCHING
      PATTERN "*.h"
      PATTERN "*.hpp")
    install(FILES vendor/cppad/x64-Release/lib/cppad_lib.lib DESTINATION ${CMAKE_INSTALL_LIBDIR})
    install(FILES vendor/cppad/x64-Debug/lib/cppad_lib.lib DESTINATION ${CMAKE_INSTALL_LIBDIR}/debug)
  endif()

  # ── CMake integration ───────────────────────────────────────────────────────── fmu4cpp source template —
  # generate_fmu.cmake needs this to configure FMU stubs
  install(FILES vendor/fmu4cpp/export/src/fmu4cpp/model_identifier.cpp.in
          DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/Aetherion/fmu4cpp/export/src/fmu4cpp)

  install(FILES cmake/generate_fmu.cmake DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/Aetherion)

  write_basic_package_version_file("${CMAKE_CURRENT_BINARY_DIR}/AetherionConfigVersion.cmake" VERSION ${PROJECT_VERSION}
                                   COMPATIBILITY SameMajorVersion)

  configure_package_config_file(cmake/AetherionConfig.cmake.in "${CMAKE_CURRENT_BINARY_DIR}/AetherionConfig.cmake"
                                INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/Aetherion)

  install(FILES "${CMAKE_CURRENT_BINARY_DIR}/AetherionConfig.cmake"
                "${CMAKE_CURRENT_BINARY_DIR}/AetherionConfigVersion.cmake"
          DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/Aetherion)

  install(
    EXPORT AetherionTargets
    FILE AetherionTargets.cmake
    NAMESPACE Aetherion::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/Aetherion)

  # ── Packaging (CPack / WiX MSI on Windows) ──────────────────────────────────
  include(cmake/Packaging.cmake)
endif()
