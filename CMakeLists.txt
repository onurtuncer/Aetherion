# ------------------------------------------------------------------------------
# Project: Aetherion
# Copyright (c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: MIT
# License-Filename: LICENSE
# ------------------------------------------------------------------------------

cmake_minimum_required(VERSION 3.26)

file(STRINGS "${CMAKE_CURRENT_SOURCE_DIR}/version.txt" projectVersion)

project(Aetherion
    VERSION     ${projectVersion}
    DESCRIPTION "Flight dynamics simulation library with FMI/FMU support"
    LANGUAGES   CXX
)

# ------------------------------------------------------------------------------
# CMake module path — must come before any include() that uses it
# ------------------------------------------------------------------------------
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

# ------------------------------------------------------------------------------
# Options — all option() declarations live here, parsed before anything else
# ------------------------------------------------------------------------------
include(cmake/AetherionOptions.cmake)

# ------------------------------------------------------------------------------
# Global compiler settings
# ------------------------------------------------------------------------------
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE  ON)

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
# Docs-only build — configure only the doc target and exit early
# ------------------------------------------------------------------------------
if(BUILD_DOCS)
    add_subdirectory(doc)
    return()
endif()


include("cmake/generate_fmu.cmake")


# ------------------------------------------------------------------------------
# Vendored dependencies
# ------------------------------------------------------------------------------
# --- fmu4cpp vendor integration ---
set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
set(FMU4CPP_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(FMU4CPP_TESTS OFF CACHE BOOL "" FORCE)
add_subdirectory(vendor/fmu4cpp/export EXCLUDE_FROM_ALL)

# ------------------------------------------------------------------------------
# Other vendored dependencies (Eigen, nlohmann_json, CppAD, ecos, …)
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
    add_subdirectory(examples)
endif()

# ------------------------------------------------------------------------------
# Install rules (optional, guarded)
# ------------------------------------------------------------------------------
#if(AETHERION_INSTALL)
#    include(GNUInstallDirs)
#
#    install(TARGETS Aetherion
#        EXPORT  AetherionTargets
#        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
#        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
#        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
#    )
#
#    install(DIRECTORY src/
#        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/Aetherion
#        FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
#    )
#
#    install(EXPORT AetherionTargets
#        FILE        AetherionTargets.cmake
#        NAMESPACE   Aetherion::
#        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/Aetherion
#    )
#endif()
