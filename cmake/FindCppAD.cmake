# ------------------------------------------------------------------------------
# Project: Aetherion Copyright (c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: MIT License-Filename: LICENSE
# ------------------------------------------------------------------------------
#
# FindCppAD -- locate CppAD (https://github.com/coin-or/CppAD).
#
# CppAD installs headers, a compiled cppad_lib and a pkg-config file, but no CMake package configuration -- hence this
# module. It is deliberately layout-agnostic: it works against a vcpkg-installed CppAD (the default for this project,
# see cmake/VcpkgToolchain.cmake), a distribution package, or any hand-built prefix on CMAKE_PREFIX_PATH.
#
# Result variables:
#
# * CppAD_FOUND             -- CppAD was located
# * CppAD_VERSION           -- e.g. 20250000.3, parsed from cppad/configure.hpp
# * CppAD_INCLUDE_DIRS      -- include directory holding cppad/cppad.hpp
# * CppAD_LIBRARIES         -- link libraries
#
# Imported target:
#
# * CppAD::cppad            -- headers plus cppad_lib, with per-configuration locations when a vcpkg-style debug/release
#   pair is installed
# ------------------------------------------------------------------------------

find_path(CppAD_INCLUDE_DIR NAMES cppad/cppad.hpp DOC "Directory containing cppad/cppad.hpp")
mark_as_advanced(CppAD_INCLUDE_DIR)

# vcpkg installs release libraries under <prefix>/lib and debug ones under <prefix>/debug/lib, and deletes
# <prefix>/debug/include -- so the include directory always identifies the prefix, for both configurations. Searching
# from there explicitly (rather than letting find_library pick whatever the current CMAKE_BUILD_TYPE puts first on the
# search path) is what keeps the two apart.
if(CppAD_INCLUDE_DIR)
  get_filename_component(_cppad_prefix "${CppAD_INCLUDE_DIR}" DIRECTORY)

  find_library(
    CppAD_LIBRARY_RELEASE
    NAMES cppad_lib
    HINTS "${_cppad_prefix}"
    PATH_SUFFIXES lib lib64
    NO_DEFAULT_PATH)

  # debug/lib is vcpkg's layout; lib/debug is the one Aetherion's own installer writes (see the CppAD install rules in
  # the top-level CMakeLists.txt), so that an installed SDK stays consumable by this same module.
  find_library(
    CppAD_LIBRARY_DEBUG
    NAMES cppad_lib
    HINTS "${_cppad_prefix}"
    PATH_SUFFIXES
      debug/lib
      debug/lib64
      lib/debug
      lib64/debug
    NO_DEFAULT_PATH)

  unset(_cppad_prefix)
endif()

# Prefix-relative lookup above misses layouts that split headers and libraries (multiarch distributions, for one), so
# fall back to a plain search before giving up.
if(NOT CppAD_LIBRARY_RELEASE AND NOT CppAD_LIBRARY_DEBUG)
  find_library(CppAD_LIBRARY_RELEASE NAMES cppad_lib)
endif()

include(SelectLibraryConfigurations)
select_library_configurations(CppAD)

if(CppAD_INCLUDE_DIR AND EXISTS "${CppAD_INCLUDE_DIR}/cppad/configure.hpp")
  file(STRINGS "${CppAD_INCLUDE_DIR}/cppad/configure.hpp" _cppad_package_string REGEX "define[ \t]+CPPAD_PACKAGE_STRING"
       LIMIT_COUNT 1)
  if(_cppad_package_string MATCHES "cppad-([0-9.]+)")
    set(CppAD_VERSION "${CMAKE_MATCH_1}")
  endif()
  unset(_cppad_package_string)
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(CppAD REQUIRED_VARS CppAD_LIBRARY CppAD_INCLUDE_DIR VERSION_VAR CppAD_VERSION)

if(CppAD_FOUND)
  set(CppAD_INCLUDE_DIRS "${CppAD_INCLUDE_DIR}")
  set(CppAD_LIBRARIES "${CppAD_LIBRARY}")

  if(NOT TARGET CppAD::cppad)
    # UNKNOWN rather than STATIC: the same module has to work for a shared cppad_lib (vcpkg's default x64-windows
    # triplet, most Linux distributions) and a static one (the -static-md triplet this project selects on Windows).
    add_library(
      CppAD::cppad
      UNKNOWN
      IMPORTED
      GLOBAL)
    set_target_properties(CppAD::cppad PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${CppAD_INCLUDE_DIR}")

    # A configuration-less IMPORTED_LOCATION is what CMake falls back to for build types it cannot map -- which includes
    # this project's sanitizer configurations. Those compile without NDEBUG, so they want the debug cppad_lib (CppAD
    # links assertion-enabled headers against an assertion-enabled library); everything else defaults to release.
    if(CppAD_LIBRARY_RELEASE)
      set_property(TARGET CppAD::cppad APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
      set_target_properties(CppAD::cppad PROPERTIES IMPORTED_LOCATION_RELEASE "${CppAD_LIBRARY_RELEASE}"
                                                    IMPORTED_LOCATION "${CppAD_LIBRARY_RELEASE}")
    endif()

    if(CppAD_LIBRARY_DEBUG)
      set_property(TARGET CppAD::cppad APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
      set_target_properties(CppAD::cppad PROPERTIES IMPORTED_LOCATION_DEBUG "${CppAD_LIBRARY_DEBUG}")

      foreach(
        _cppad_sanitizer_config
        AddressSanitizer
        MemorySanitizer
        ThreadSanitizer
        UndefinedBehaviorSanitizer)
        string(TOUPPER "${_cppad_sanitizer_config}" _cppad_sanitizer_config)
        set_property(TARGET CppAD::cppad PROPERTY MAP_IMPORTED_CONFIG_${_cppad_sanitizer_config} Debug)
      endforeach()
      unset(_cppad_sanitizer_config)
    endif()

    if(NOT CppAD_LIBRARY_RELEASE)
      set_target_properties(CppAD::cppad PROPERTIES IMPORTED_LOCATION "${CppAD_LIBRARY_DEBUG}")
    endif()
  endif()
endif()
