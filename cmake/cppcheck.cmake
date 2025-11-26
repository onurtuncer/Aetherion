# ------------------------------------------------------------------------------
# Copyright (c) 2025 Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: MIT
# License-Filename: LICENSE
# ------------------------------------------------------------------------------

# Skip static analysis on Windows completely
if (WIN32)
    message(STATUS "Skipping cppcheck static analysis target on Windows.")
    return()
endif()

# Try to find cppcheck
find_program(CPPCHECK_EXECUTABLE NAMES cppcheck)

if (NOT CPPCHECK_EXECUTABLE)
    message(STATUS "cppcheck not found - 'static_analysis' target will NOT be created.")
    return()
endif()

file(GLOB_RECURSE APP_SOURCES "${CMAKE_SOURCE_DIR}/src/*.cpp")

set(STATIC_ANALYSIS_SOURCE_DIR "${CMAKE_SOURCE_DIR}")
set(STATIC_ANALYSIS_REPORT_DIR "${CMAKE_BINARY_DIR}/static_analysis")
file(MAKE_DIRECTORY "${STATIC_ANALYSIS_REPORT_DIR}")
set(STATIC_ANALYSIS_XML_FILE "${STATIC_ANALYSIS_REPORT_DIR}/static_analysis_report.xml")

set(NAMESPACED_TARGET "static_analysis")
add_custom_target(${NAMESPACED_TARGET}
    COMMAND "${CPPCHECK_EXECUTABLE}"
        --enable=all
        --force
        --suppress=unusedFunction
        --cppcheck-build-dir="${STATIC_ANALYSIS_REPORT_DIR}"
        -I"${CMAKE_SOURCE_DIR}"
        --suppressions-list="${CMAKE_BINARY_DIR}/../../tools/cppcheck/CppCheckSuppressions.txt"
        --inconclusive
        --xml-version=2
        -v ${APP_SOURCES}
        --output-file="${STATIC_ANALYSIS_XML_FILE}"
    COMMAND cppcheck-htmlreport
        --title="${CMAKE_PROJECT_NAME}"
        --file="${STATIC_ANALYSIS_XML_FILE}"
        --report-dir="${STATIC_ANALYSIS_REPORT_DIR}"
        --source-dir="${STATIC_ANALYSIS_SOURCE_DIR}"
        --source-encoding=UTF-8
    COMMENT "Running cppcheck static analysis & generating HTML report"
    VERBATIM
)
