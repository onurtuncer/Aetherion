# ------------------------------------------------------------------------------
# Project: Aetherion
# Copyright (c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
#
# SPDX-License-Identifier: MIT
# License-Filename: LICENSE
# ------------------------------------------------------------------------------

add_executable(test_spatial
                    test_Inertia.cpp 
                    test_Skew.cpp 
                    test_Transform.cpp 
                    test_CrossForce.cpp
                    test_CrossMotion.cpp 
                    test_Adjoint.cpp)

target_compile_options(test_spatial PRIVATE $<$<CXX_COMPILER_ID:MSVC>:/utf-8>)

target_link_libraries(test_spatial
    PRIVATE
        Aetherion
        Catch2::Catch2WithMain
        CppAD::cppad  
        Eigen3::Eigen
)

add_test(NAME spatial_algebra_tests COMMAND test_spatial)