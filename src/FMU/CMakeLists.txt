# ------------------------------------------------------------------------------
# Project: Aetherion
# Copyright (c) 2025, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: MIT
# License-Filename: LICENSE
# ------------------------------------------------------------------------------

# src/FMU/CMakeLists.txt

# assume generate_fmu() is already available from top-level include

# Identity FMU (your FeedThrough model)
generateFMU(
    TARGET FeedThrough
    FMI_VERSIONS fmi2
    SOURCES
        FeedThrough/FeedThrough.cpp
)
