# SPDX-License-Identifier: MIT

# Instrument only test_estimation: the pilot's production code is header-only.
include_guard(GLOBAL)

if(NOT
   CMAKE_CXX_COMPILER_ID
   STREQUAL
   "Clang"
   OR NOT
      CMAKE_CXX_COMPILER_VERSION
      VERSION_GREATER_EQUAL
      19
   OR CMAKE_CXX_COMPILER_VERSION VERSION_GREATER_EQUAL 20)
  message(FATAL_ERROR "Mutation testing requires Clang 19 and Mull 19; use a separate build directory.")
endif()
if(NOT
   CMAKE_SYSTEM_NAME
   STREQUAL
   "Linux")
  message(FATAL_ERROR "The mutation pilot currently supports Linux only.")
endif()
if(AETHERION_ENABLE_COVERAGE)
  message(FATAL_ERROR "Use separate builds for gcov coverage and Mull mutation testing.")
endif()

set(AETHERION_MULL_PLUGIN "/usr/lib/mull-ir-frontend-19" CACHE FILEPATH "Mull LLVM 19 frontend plugin")
if(NOT EXISTS "${AETHERION_MULL_PLUGIN}")
  message(FATAL_ERROR "Missing Mull plugin: ${AETHERION_MULL_PLUGIN}")
endif()
find_program(AETHERION_MULL_RUNNER NAMES mull-runner-19 REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

set(_mutation_config "${PROJECT_SOURCE_DIR}/tools/mutation/mull.yml")
# Set the config for compilation too, independent of the build working directory.
set_property(TARGET test_estimation PROPERTY CXX_COMPILER_LAUNCHER
                                            "${CMAKE_COMMAND};-E;env;MULL_CONFIG=${_mutation_config}")
target_compile_options(
  test_estimation
  PRIVATE -O0
          -g
          -grecord-command-line
          "-fpass-plugin=${AETHERION_MULL_PLUGIN}")
# Mutators/path filters are applied during compilation: changing them must rebuild.
get_target_property(_mutation_sources test_estimation SOURCES)
set_property(SOURCE ${_mutation_sources} APPEND PROPERTY OBJECT_DEPENDS "${_mutation_config}")

add_custom_target(
  mutation
  COMMAND
    "${Python3_EXECUTABLE}" "${PROJECT_SOURCE_DIR}/tools/mutation/run.py" --binary "$<TARGET_FILE:test_estimation>"
    --runner "${AETHERION_MULL_RUNNER}" --output "${CMAKE_BINARY_DIR}/mutation-report"
  DEPENDS test_estimation
  WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}"
  COMMENT "Run replay mutation-testing baseline and Mull campaign"
  USES_TERMINAL VERBATIM)
