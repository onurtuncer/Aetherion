# ------------------------------------------------------------------------------
# Project: Aetherion
# Copyright (c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: MIT
# License-Filename: LICENSE
# ------------------------------------------------------------------------------
# src/Coordinate/CMakeLists.txt
#
# Registers Coordinate module sources into the Aetherion library target.
#
# Modules:
#   ECEFtoGeodetic   — ECEF to geodetic (lat/lon/alt) conversion
#   InertialToLocal  — Inertial (ECI) to local (NED) frame transformation
#
# Note: ECEFtoGeodetic.cpp is listed but its corresponding call-site in the
# root CMakeLists.txt was commented out (#ECEFToGeodetic.cpp). It is included
# here unconditionally — remove or guard it with an option if the
# implementation is still experimental.
# ------------------------------------------------------------------------------

target_sources(Aetherion PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/ECEFtoGeodetic.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/InertialToLocal.cpp
)