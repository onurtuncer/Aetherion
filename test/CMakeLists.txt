# Enable testing
include(CTest)

# -----------------------
# Catch2 via FetchContent
# -----------------------
include(FetchContent)

FetchContent_Declare(
    catch2
    GIT_REPOSITORY https://github.com/catchorg/Catch2.git
    GIT_TAG        v3.5.0
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
        ode::ode          # <-- libode target from FetchContent
)

add_test(NAME libode_basic_test COMMAND test_libode)