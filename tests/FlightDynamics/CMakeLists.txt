# ------------------------------------------------------------------------------
# Project: Aetherion
# Copyright (c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: MIT
# License-Filename: LICENSE
# ------------------------------------------------------------------------------

add_executable(test_initial_state 
                    test_BuildInitialStateVector.cpp)

target_link_libraries(test_initial_state
    PRIVATE
        Aetherion
        Catch2::Catch2WithMain
)

include(Catch)
catch_discover_tests(test_initial_state)

