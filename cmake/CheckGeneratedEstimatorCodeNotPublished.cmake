# ------------------------------------------------------------------------------
# Project: Aetherion Copyright (c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: MIT License-Filename: LICENSE
# ------------------------------------------------------------------------------
#
# Generated estimator C is never published from this repository (TODO-sensor-fusion.md, "Decided (2026-11-13)"): not
# committed, not installed, not shipped, and neither is the generator executable. This script is the check that belongs
# with the generator tool; CTest runs it as estimation_generated_code_not_published.
#
# cmake -DSOURCE_DIR=<repository> -DBINARY_DIR=<build tree> -P CheckGeneratedEstimatorCodeNotPublished.cmake
#
# 1. No file in the git index carries the marker line generate_ekf_c writes into each generated file. Skipped, with a
#    message, outside a git checkout.
# 2. No install script or install manifest of the build tree names the generated directories, the generated files or the
#    generator executables.

if(NOT SOURCE_DIR OR NOT BINARY_DIR)
  message(FATAL_ERROR "usage: cmake -DSOURCE_DIR=<repository> -DBINARY_DIR=<build tree> -P <this script>")
endif()

# Assembled, so that this file does not contain the marker itself.
string(CONCAT _marker "AETHERION-GENERATED-" "ESTIMATOR-CODE")
set(_problems "")

find_program(_git git)
if(_git AND EXISTS "${SOURCE_DIR}/.git")
  execute_process(
    COMMAND "${_git}" -C "${SOURCE_DIR}" grep --cached -l -F -e "${_marker}"
    OUTPUT_VARIABLE _tracked
    RESULT_VARIABLE _result
    OUTPUT_STRIP_TRAILING_WHITESPACE)
  # git grep: 0 = found, 1 = nothing found, anything else = it could not run.
  if(_result EQUAL 0)
    string(APPEND _problems "generated estimator code is in the git index:\n${_tracked}\n")
  elseif(
    NOT
    _result
    EQUAL
    1)
    string(APPEND _problems "git grep failed (${_result}); the index was not checked\n")
  endif()
else()
  message(STATUS "Not a git checkout (or no git): the index was not checked for generated estimator code")
endif()

set(_forbidden
    generated_ekf
    generated_cg
    aeth_ekf
    estimation_cg
    generate_ekf_c
    generate_estimation_cg)
file(
  GLOB_RECURSE
  _install_files
  "${BINARY_DIR}/cmake_install.cmake"
  "${BINARY_DIR}/install_manifest*.txt")
foreach(_file IN LISTS _install_files)
  file(READ "${_file}" _text)
  foreach(_name IN LISTS _forbidden)
    string(FIND "${_text}" "${_name}" _at)
    if(NOT
       _at
       EQUAL
       -1)
      string(APPEND _problems
             "${_file} names '${_name}': generated estimator code or its generator would be installed\n")
    endif()
  endforeach()
endforeach()

if(_problems)
  message(FATAL_ERROR "Generated estimator code must not be published from this repository:\n${_problems}")
endif()
list(LENGTH _install_files _count)
message(STATUS "No generated estimator code in the git index or in ${_count} install scripts and manifests")
