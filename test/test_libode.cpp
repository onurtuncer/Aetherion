#define CATCH_CONFIG_MAIN
#include <catch2/catch.hpp>

#include <ode/ode_rk_4.h>   // libode RK4 solver
#include <cmath>

TEST_CASE("libode RK4 integrates exp growth", "[libode]") {
    using namespace ode;

    // Define dx/dt = f(t, x)
    auto f = [](double /*t*/, const double* x, double* dxdt) {
        dxdt[0] = x[0];
        };

    // State vector
    double x[1] = { 1.0 };

    // Create RK4 solver for dimension = 1
    OdeRK4 solver(1);

    // Integrate from t=0 with dt = 0.1 for 10 steps
    double t = 0.0;
    double dt = 0.1;

    for (int i = 0; i < 10; ++i) {
        solver.step(f, t, x, dt);
        t += dt;
    }

    // Compare with analytical e^1
    double expected = std::exp(1.0);
    REQUIRE(x[0] == Approx(expected).epsilon(1e-3));
}
