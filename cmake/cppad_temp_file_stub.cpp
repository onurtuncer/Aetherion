// Stub implementation of CppAD::local::temp_file for header-only builds.
//
// The real implementation lives in cppad_lib (the compiled CppAD library).
// It creates a temporary binary file so NaN debug data can be inspected.
//
// On Linux we use CppAD header-only (to avoid its CMakeLists conflicting
// with CTest's reserved "test" target), so cppad_lib is never built.
// This stub satisfies the linker; it returns an empty path, meaning
// put_check_for_nan writes to "" which silently fails — acceptable because
// that code path is only reached when a Forward/Reverse call produces NaN,
// which must not happen in a passing test suite.
#include <string>
namespace CppAD { namespace local {
    std::string temp_file() { return std::string{}; }
} }
