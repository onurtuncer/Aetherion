# ------------------------------------------------------------------------------
# Project: Aetherion
# Copyright (c) 2025, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: MIT
# License-Filename: LICENSE
# ------------------------------------------------------------------------------

add_executable(test_aero
                    test_MachNumber.cpp 
                    test_AerodynamicAngles.cpp
                    test_AerodynamicForces.cpp 
                    test_AerodynamicMoments.cpp 
                    "../RigidBody/test_AerodynamicWrench.cpp")

target_link_libraries(test_aero
    PRIVATE
        Aetherion
        Catch2::Catch2WithMain
        CppAD::cppad  
        Eigen3::Eigen
)

target_include_directories(test_aero PRIVATE "${CMAKE_SOURCE_DIR}/vendor/CppAD/include")

include(Catch)
catch_discover_tests(test_aero)
                                               
#add_test(NAME aero_tests COMMAND test_aero)