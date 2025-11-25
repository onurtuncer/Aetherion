# ------------------------------------------------------------------------------
# Project: Aetherion
# Copyright (c) 2025, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: MIT
# License-Filename: LICENSE
# ------------------------------------------------------------------------------

# -----------------------
# Catch2 via FetchContent
# -----------------------

FetchContent_Declare(
    catch2
    GIT_REPOSITORY https://github.com/catchorg/Catch2.git
    GIT_TAG        v3.5.2
)
FetchContent_MakeAvailable(catch2)

# -----------------------
# Test executable
# -----------------------
add_executable(test_libode
    test_libode.cpp
)

target_link_libraries(test_libode
    PRIVATE
        Catch2::Catch2WithMain
        libode::libode
)

add_test(NAME libode_basic_test COMMAND test_libode)

add_executable(test_cppad
    test_cppad.cpp
)

target_link_libraries(test_cppad
    PRIVATE
        Catch2::Catch2WithMain
        CppAD::cppad      
)

add_test(NAME cppad_basic_test COMMAND test_cppad)

add_executable(test_eigen
    test_eigen.cpp
)

target_link_libraries(test_eigen
    PRIVATE
        Catch2::Catch2WithMain
        Eigen::Eigen       
)

add_test(NAME eigen_basic_test COMMAND test_eigen)