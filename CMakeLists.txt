# ------------------------------------------------------------------------------
# Project: Aetherion
# Copyright (c) 2025, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: MIT
# License-Filename: LICENSE
# ------------------------------------------------------------------------------

cmake_minimum_required(VERSION 3.26)

file(STRINGS "${CMAKE_CURRENT_SOURCE_DIR}/version.txt" projectVersion)
project(Aetherion VERSION ${projectVersion} LANGUAGES CXX)

option(FMU4CPP_BUILD_TESTS "Build internal tests" ON)
option(FMU4CPP_MSVC_STATIC_CRT "Use static MSVC runtime library" ON)

set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

if (BUILD_DOCS)
  add_subdirectory(doc)
  return()
endif()

include("cmake/generate_fmu.cmake")

add_subdirectory(${CMAKE_SOURCE_DIR}/export)



include(Dependencies)

add_subdirectory(src/FMU)

if(NOT BUILD_DOCS)

add_library(Aetherion INTERFACE)
    
target_include_directories(Aetherion
    INTERFACE
        $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}>
      #  $<INSTALL_INTERFACE:include>
)

add_executable(aetherion
    src/main.cpp
    # add other .cpp files here
 )


target_link_libraries(aetherion
    PRIVATE
       # fmu4cpp::fmu4cpp
        ode::ode        
)


if (ENABLE_STATIC_ANALYSIS)
  include(cppcheck)
endif()

if(AETHERION_BUILD_TESTS)
   
	add_subdirectory(tests)
endif()

endif() # Build docs

