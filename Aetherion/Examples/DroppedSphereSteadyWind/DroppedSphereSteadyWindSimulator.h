// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------

#pragma once

#include <Aetherion/Examples/DroppedSphereSteadyWind/DroppedSphereSteadyWindTypes.h>
#include <Aetherion/Simulation/ISimulator.h>
#include <Aetherion/Simulation/MakeSnapshot1.h>
#include <Aetherion/Simulation/Snapshot1.h>
#include <Aetherion/RigidBody/InertialParameters.h>
#include <Aetherion/RigidBody/AerodynamicParameters.h>
#include <Aetherion/Environment/Wind.h>
#include <Aetherion/ODE/RKMK/Core/NewtonOptions.h>

namespace Aetherion::Examples::DroppedSphereSteadyWind {

class DroppedSphereSteadyWindSimulator
    : public Simulation::ISimulator<DroppedSphereSteadyWindVF, Simulation::Snapshot1>
{
public:
    static constexpr double kOmegaEarth_rad_s = 7.2921150e-5;

    explicit DroppedSphereSteadyWindSimulator(
        const RigidBody::InertialParameters&    ip,
        const RigidBody::AerodynamicParameters& aero,
        const Environment::ConstantWind&        wind,
        double                                  lat0_rad,
        double                                  lon0_rad,
        RigidBody::StateD                       initialState,
        double                                  theta0_rad = 0.0,
        ODE::RKMK::Core::NewtonOptions          opt = {})
        : ISimulator<DroppedSphereSteadyWindVF, Simulation::Snapshot1>(
            DroppedSphereSteadyWindVF(
                ip,
                FlightDynamics::J2GravityPolicy{},
                makeDragPolicy(aero, wind, lat0_rad, lon0_rad, theta0_rad)),
            std::move(initialState), opt)
        , m_Theta0(theta0_rad)
    {
    }

    [[nodiscard]]
    Simulation::Snapshot1 snapshot() const noexcept override
    {
        return Simulation::MakeSnapshot1(
            time(), state(), currentTheta(), vectorField().gravity);
    }

    [[nodiscard]]
    double currentTheta() const noexcept
    {
        return m_Theta0 + kOmegaEarth_rad_s * time();
    }

protected:
    void validate() const override {}

private:
    double m_Theta0;

    /// Convert NED wind → ECEF at the initial launch position, then construct policy.
    static FlightDynamics::SteadyWindDragPolicy
        makeDragPolicy(const RigidBody::AerodynamicParameters& aero,
                       const Environment::ConstantWind&        wind,
                       double lat0_rad, double lon0_rad,
                       double theta0_rad)
    {
        // NED-to-ECEF at (lat0, lon0):
        //   N_ecef = (-sin(lat)*cos(lon), -sin(lat)*sin(lon),  cos(lat))
        //   E_ecef = (-sin(lon),           cos(lon),            0       )
        //   D_ecef = (-cos(lat)*cos(lon), -cos(lat)*sin(lon), -sin(lat) )
        const double sLat = std::sin(lat0_rad), cLat = std::cos(lat0_rad);
        const double sLon = std::sin(lon0_rad), cLon = std::cos(lon0_rad);

        const double Nx = -sLat*cLon, Ny = -sLat*sLon, Nz =  cLat;
        const double Ex = -sLon,      Ey =  cLon,      Ez =  0.0;
        const double Dx = -cLat*cLon, Dy = -cLat*sLon, Dz = -sLat;

        const double vN = wind.north_mps, vE = wind.east_mps, vD = wind.down_mps;

        // Wind in ECEF
        double wx_ecef = vN*Nx + vE*Ex + vD*Dx;
        double wy_ecef = vN*Ny + vE*Ey + vD*Dy;
        double wz_ecef = vN*Nz + vE*Ez + vD*Dz;

        // ECEF → ECI at theta0 (rotate by +theta0 around z)
        const double ct = std::cos(theta0_rad), st = std::sin(theta0_rad);
        const double wx_eci = ct*wx_ecef - st*wy_ecef;
        const double wy_eci = st*wx_ecef + ct*wy_ecef;
        const double wz_eci = wz_ecef;

        // Store as ECEF (the policy rotates back to ECI at runtime using ERA)
        return FlightDynamics::SteadyWindDragPolicy(
            aero.CD, aero.S, wx_ecef, wy_ecef, wz_ecef);
    }
};

} // namespace Aetherion::Examples::DroppedSphereSteadyWind
