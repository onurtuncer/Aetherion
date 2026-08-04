# ------------------------------------------------------------------------------
# Project: Aetherion Copyright (c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: MIT License-Filename: LICENSE
# ------------------------------------------------------------------------------
#
# vcpkg toolchain auto-selection.
#
# Included from the top-level CMakeLists.txt *before* project(), which is the last point at which CMAKE_TOOLCHAIN_FILE
# can still be set. Dependencies declared in vcpkg.json (currently CppAD) are then installed by vcpkg at configure time
# into ${CMAKE_BINARY_DIR}/vcpkg_installed and picked up by the ordinary find_package() calls in Dependencies.cmake --
# one code path on every platform.
#
# Every decision made here can be overridden:
#
# * -DCMAKE_TOOLCHAIN_FILE=...    use a different toolchain (vcpkg is then not touched)
# * -DAETHERION_VCPKG_ROOT=...    point at a specific vcpkg clone
# * -DAETHERION_USE_VCPKG=OFF     do not use vcpkg; dependencies must be discoverable on CMAKE_PREFIX_PATH
# * -DVCPKG_TARGET_TRIPLET=...    override the triplet chosen below
#
# Nothing here is fatal: without vcpkg the find_package() calls simply fall back to a system-installed CppAD and report
# a helpful error if there is none.
# ------------------------------------------------------------------------------

option(AETHERION_USE_VCPKG "Resolve third-party dependencies through vcpkg" ON)
set(AETHERION_VCPKG_ROOT ""
    CACHE PATH "vcpkg clone to use (overrides the VCPKG_ROOT / VCPKG_INSTALLATION_ROOT environment variables)")

if(NOT AETHERION_USE_VCPKG OR BUILD_DOCS)
  return()
endif()

if(DEFINED CMAKE_TOOLCHAIN_FILE OR DEFINED ENV{CMAKE_TOOLCHAIN_FILE})
  # Caller supplied a toolchain (possibly vcpkg's own, possibly a cross-compilation one that chainloads it) -- respect
  # it and stay out of the way.
  return()
endif()

# VCPKG_ROOT is what a local clone sets; VCPKG_INSTALLATION_ROOT is what GitHub-hosted runners export for their
# preinstalled vcpkg. Checking both keeps CI working with no workflow-side setup.
#
# The ports/ check is not paranoia: vcvarsall.bat (and therefore every VS developer prompt, and the msvc-dev-cmd action
# used in CI) overwrites VCPKG_ROOT with the stub vcpkg shipped inside Visual Studio, which carries a vcpkg.exe and a
# toolchain file but no port registry at all. Selecting it produces an installation that silently resolves nothing.
foreach(_aetherion_vcpkg_root "${AETHERION_VCPKG_ROOT}" "$ENV{VCPKG_ROOT}" "$ENV{VCPKG_INSTALLATION_ROOT}")
  if(NOT _aetherion_vcpkg_root)
    continue()
  endif()

  if(NOT EXISTS "${_aetherion_vcpkg_root}/scripts/buildsystems/vcpkg.cmake")
    message(STATUS "[Aetherion] Skipping vcpkg root '${_aetherion_vcpkg_root}': no toolchain file")
    continue()
  endif()

  if(NOT IS_DIRECTORY "${_aetherion_vcpkg_root}/ports")
    message(STATUS "[Aetherion] Skipping vcpkg root '${_aetherion_vcpkg_root}': no port registry (ports/)")
    continue()
  endif()

  set(CMAKE_TOOLCHAIN_FILE "${_aetherion_vcpkg_root}/scripts/buildsystems/vcpkg.cmake"
      CACHE FILEPATH "Toolchain file (vcpkg, auto-detected by Aetherion)")
  message(STATUS "[Aetherion] Using vcpkg toolchain: ${CMAKE_TOOLCHAIN_FILE}")
  break()
endforeach()

unset(_aetherion_vcpkg_root)

if(NOT DEFINED CMAKE_TOOLCHAIN_FILE)
  message(STATUS "[Aetherion] No usable vcpkg found -- searching for dependencies on the system instead. "
                 "Point AETHERION_VCPKG_ROOT (or the VCPKG_ROOT environment variable) at a vcpkg clone, "
                 "or configure with -DAETHERION_USE_VCPKG=OFF to silence this.")
  return()
endif()

# Static dependencies with the dynamic CRT: Aetherion is a static library whose FMUs and examples ship as self-contained
# binaries, so a cppad_lib.dll next to every executable would be pure overhead. -static-md (rather than -static) keeps
# the /MD runtime that MSVC and the rest of this build use by default.
if(CMAKE_HOST_SYSTEM_NAME STREQUAL "Windows" AND NOT DEFINED VCPKG_TARGET_TRIPLET)
  if(CMAKE_HOST_SYSTEM_PROCESSOR STREQUAL "ARM64")
    set(VCPKG_TARGET_TRIPLET "arm64-windows-static-md" CACHE STRING "vcpkg triplet")
  else()
    set(VCPKG_TARGET_TRIPLET "x64-windows-static-md" CACHE STRING "vcpkg triplet")
  endif()
endif()
