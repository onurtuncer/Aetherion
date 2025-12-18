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
                    test_AerodynamicMoments.cpp)

target_link_libraries(test_aero
    PRIVATE
        Aetherion
        Catch2::Catch2WithMain
        CppAD::cppad  
        Eigen3::Eigen
)

target_include_directories(test_aero PRIVATE "${CMAKE_SOURCE_DIR}/vendor/CppAD/include")
                                               
add_test(NAME aero_tests COMMAND test_aero)