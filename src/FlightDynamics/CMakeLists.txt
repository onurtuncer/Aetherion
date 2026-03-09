# ------------------------------------------------------------------------------
# Project: Aetherion
# Copyright (c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: MIT
# License-Filename: LICENSE
# ------------------------------------------------------------------------------
# src/FlightDynamics/CMakeLists.txt
#
# Registers FlightDynamics module sources into the Aetherion library target
# and delegates to submodule CMakeLists files where appropriate.
#
# Structure:
#   FlightDynamics/
#   └── Serialization/     — JSON (de)serialization via nlohmann adapter
#
# Future submodules (Aerodynamics, RigidBody, Environment, etc.) should each
# get their own subdirectory and CMakeLists.txt following the same pattern,
# then be added with add_subdirectory() below.
# ------------------------------------------------------------------------------

add_subdirectory(Serialization)

# --- Future submodules --------------------------------------------------------
# add_subdirectory(Aerodynamics)
# add_subdirectory(Environment)
# add_subdirectory(RigidBody)