# ------------------------------------------------------------------------------
# Project: Aetherion Copyright (c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: MIT License-Filename: LICENSE
# ------------------------------------------------------------------------------

add_executable(test_estimation test_Configuration.cpp test_LogContract.cpp)

target_link_libraries(test_estimation PRIVATE Aetherion Catch2::Catch2WithMain)

include(Catch)
catch_discover_tests(test_estimation)
