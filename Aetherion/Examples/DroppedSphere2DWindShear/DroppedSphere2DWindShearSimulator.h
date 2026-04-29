// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------

#pragma once

#include <Aetherion/Examples/DroppedSphere2DWindShear/DroppedSphere2DWindShearTypes.h>
#include <Aetherion/Simulation/ISimulator.h>
#include <Aetherion/Simulation/MakeSnapshot1.h>
#include <Aetherion/Simulation/Snapshot1.h>
#include <Aetherion/RigidBody/InertialParameters.h>
#include <Aetherion/RigidBody/AerodynamicParameters.h>
#include <Aetherion/Environment/Wind.h>
#include <Aetherion/ODE/RKMK/Core/NewtonOptions.h>
#include <Eigen/Dense>

namespace Aetherion::Examples::DroppedSphere2DWindShear {

class DroppedSphere2DWindShearSimulator
    : public Simulation::ISimulator<DroppedSphere2DWindShearVF, Simulation::Snapshot1>
{
public:
    static constexpr double kOmegaEarth_rad_s = 7.2921150e-5;

    explicit DroppedSphere2DWindShearSimulator(
        const RigidBody::InertialParameters&    ip,
        const RigidBody::AerodynamicParameters& aero,
        const Environment::WindShear&           ws,
        double                                  lat0_rad,
        double                                  lon0_rad,
        RigidBody::StateD                       initialState,
        double                                  theta0_rad = 0.0,
        ODE::RKMK::Core::NewtonOptions          opt = {})
        : ISimulator<DroppedSphere2DWindShearVF, Simulation::Snapshot1>(
            DroppedSphere2DWindShearVF(
                ip,
                FlightDynamics::J2GravityPolicy{},
                makeAeroPolicy(aero, ws, lat0_rad, lon0_rad)),
            std::move(initialState), opt)
        , m_Theta0(theta0_rad)
        , m_WindNED(ws.north_ref_mps, ws.east_ref_mps, ws.down_ref_mps)
    {
    }

    [[nodiscard]]
    Simulation::Snapshot1 snapshot() const noexcept override
    {
        auto snap = Simulation::MakeSnapshot1(
            time(), state(), currentTheta(), vectorField().gravity);

        // Correct TAS to wind-relative airspeed (same correction as Scenario 7).
        // The reference altitude wind is stored in m_WindNED; we use it as an
        // approximation since the sphere barely moves horizontally.
        const Eigen::Vector3d v_air = snap.feVelocity_m_s - m_WindNED
            * std::pow(std::max(snap.altitudeMsl_m, 0.0) /
                       vectorField().aero.wind.h_ref_m,
                       vectorField().aero.wind.shear_exp);
        snap.trueAirspeed_m_s   = v_air.norm();
        snap.mach                = snap.trueAirspeed_m_s / snap.speedOfSound_m_s;
        snap.dynamicPressure_Pa  = 0.5 * snap.airDensity_kg_m3
                                       * snap.trueAirspeed_m_s
                                       * snap.trueAirspeed_m_s;
        return snap;
    }

    [[nodiscard]]
    double currentTheta() const noexcept
    {
        return m_Theta0 + kOmegaEarth_rad_s * time();
    }

protected:
    void validate() const override {}

private:
    double          m_Theta0;
    Eigen::Vector3d m_WindNED;  ///< NED wind at reference altitude (for TAS correction)

    static ShearWindPolicy makeAeroPolicy(
        const RigidBody::AerodynamicParameters& aero,
        const Environment::WindShear&           ws,
        double lat0_rad, double lon0_rad)
    {
        auto plws = FlightDynamics::PowerLawWindShear::from_ned(
            ws.north_ref_mps, ws.east_ref_mps, ws.down_ref_mps,
            lat0_rad, lon0_rad,
            ws.h_ref_m, ws.shear_exp);
        return ShearWindPolicy{ aero.CD, aero.S, std::move(plws) };
    }
};

} // namespace Aetherion::Examples::DroppedSphere2DWindShear
