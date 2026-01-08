# ------------------------------------------------------------------------------
# Project: Aetherion
# Copyright (c) 2025, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: MIT
# License-Filename: LICENSE
# ------------------------------------------------------------------------------

include(FetchContent)

FetchContent_Declare(
    catch2
    GIT_REPOSITORY https://github.com/catchorg/Catch2.git
    GIT_TAG        v3.5.2
)
FetchContent_MakeAvailable(catch2)


include(CTest)

add_subdirectory("ThirdPartyIntegration")
add_subdirectory("Environment")
add_subdirectory("Coordinate")
add_subdirectory("ODE")
add_subdirectory("Aerodynamics")
add_subdirectory("FlightDynamics")


