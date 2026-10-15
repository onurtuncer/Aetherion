// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// Installed-SDK consumer: uses a header-only part (atmosphere through CppAD),
// a compiled part (the JSON configuration loader, which also pulls
// <nlohmann/json.hpp> from the installation), and exits nonzero on any
// mismatch. See CMakeLists.txt next to this file.
// ------------------------------------------------------------------------------

#include <cmath>
#include <cstdio>
#include <vector>

#include <cppad/cppad.hpp>
#include <nlohmann/json.hpp>

#include <Aetherion/Environment/Atmosphere.h>
#include <Aetherion/RigidBody/Config.h>
#include <Aetherion/Serialization/LoadConfig.h>

int main()
{
    int failures = 0;

    // Header-only, through CppAD: density and its derivative at 3 km.
    using AD = CppAD::AD<double>;
    std::vector<AD> h{ AD(3000.0) };
    CppAD::Independent(h);
    std::vector<AD> rho{ Aetherion::Environment::US1976Atmosphere(h[0]).rho };
    CppAD::ADFun<double> f(h, rho);
    const double r = f.Forward(0, std::vector<double>{ 3000.0 })[0];
    const double drdh = f.Jacobian(std::vector<double>{ 3000.0 })[0];
    std::printf("rho(3000 m) = %.6f kg/m^3, drho/dh = %.3e\n", r, drdh);
    if (std::abs(r - 0.909253) > 1e-5 || !(drdh < 0.0)) ++failures;

    // Compiled library and the installed JSON header: a malformed document is
    // rejected with an exception from libAetherion.
    try {
        (void)Aetherion::Serialization::LoadConfig(nlohmann::json::object(), "consumer");
        std::printf("LoadConfig accepted an empty document\n");
        ++failures;
    } catch (const std::exception& e) {
        std::printf("LoadConfig rejected an empty document: OK\n");
    }

    std::printf(failures == 0 ? "installed SDK: OK\n" : "installed SDK: %d failure(s)\n", failures);
    return failures == 0 ? 0 : 1;
}
