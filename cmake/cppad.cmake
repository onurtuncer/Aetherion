# cppad.cmake
add_library(CppAD::cppad INTERFACE IMPORTED)

set_target_properties(CppAD::cppad PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES
        "${CMAKE_CURRENT_SOURCE_DIR}/vendor/cppad/include"
)
