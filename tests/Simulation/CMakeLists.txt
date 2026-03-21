# ------------------------------------------------------------------------------
# Project: Aetherion
# Copyright (c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: MIT
# License-Filename: LICENSE
# ------------------------------------------------------------------------------

add_executable(test_snapshot          
                    test_Snapshot1.cpp
                   )

target_link_libraries(test_snapshot
    PRIVATE
        Aetherion
        Catch2::Catch2WithMain
)

include(Catch)
catch_discover_tests(test_snapshot)