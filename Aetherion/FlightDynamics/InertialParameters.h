// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX - License - Identifier: MIT
// License - Filename: LICENSE
// ------------------------------------------------------------------------------

#pragma once

namespace Aetherion::FlightDynamics {

    struct InertialParameters
    {
        // --- Mass ---
        double mass_kg{ 0.0 };

        // --- Inertia tensor about CoG, expressed in BODY frame ---
        // | Ixx  -Ixy  -Ixz |
        // | -Ixy  Iyy  -Iyz |
        // | -Ixz -Iyz   Izz |
        double Ixx{ 0.0 };
        double Iyy{ 0.0 };
        double Izz{ 0.0 };

        double Ixy{ 0.0 };
        double Iyz{ 0.0 };
        double Ixz{ 0.0 };

        // --- Body frame origin relative to center of gravity ---
        // Position of BODY axes origin w.r.t. CoG
        double xbar_m{ 0.0 };
        double ybar_m{ 0.0 };
        double zbar_m{ 0.0 };
    };

} // namespace Aetherion::FlightDynamics
