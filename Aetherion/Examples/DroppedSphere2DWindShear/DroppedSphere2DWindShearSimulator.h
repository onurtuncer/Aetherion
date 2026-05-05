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
        , m_Grad_N(ws.gradient_N_mps_m), m_Grad_E(ws.gradient_E_mps_m)
        , m_Int_N(ws.intercept_N_mps),   m_Int_E(ws.intercept_E_mps)
    {
    }

    [[nodiscard]]
    Simulation::Snapshot1 snapshot() const noexcept override
    {
        auto snap = Simulation::MakeSnapshot1(
            time(), state(), currentTheta(),
            vectorField().gravity, vectorField().aero);

        // Correct TAS to wind-relative airspeed.
        // Linear shear: v_wind_E(h) = gradient_E * h + intercept_E
        const double h = std::max(snap.altitudeMsl_m, 0.0);
        const double vE_wind = m_Grad_E * h + m_Int_E;
        const double vN_wind = m_Grad_N * h + m_Int_N;
        const Eigen::Vector3d v_wind_ned(vN_wind, vE_wind, 0.0);
        const Eigen::Vector3d v_air = snap.feVelocity_m_s - v_wind_ned;
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
    double m_Theta0;
    double m_Grad_N, m_Grad_E;  ///< NED wind altitude gradients [m/s per m]
    double m_Int_N,  m_Int_E;   ///< NED wind intercepts at h=0 [m/s]

    static ShearWindPolicy makeAeroPolicy(
        const RigidBody::AerodynamicParameters& aero,
        const Environment::WindShear&           ws,
        double lat0_rad, double lon0_rad)
    {
        auto lws = FlightDynamics::LinearWindShear::from_ned(
            ws.gradient_N_mps_m, ws.gradient_E_mps_m,
            ws.intercept_N_mps,  ws.intercept_E_mps,
            lat0_rad, lon0_rad);
        return ShearWindPolicy{ aero.CD, aero.S, std::move(lws) };
    }
};

} // namespace Aetherion::Examples::DroppedSphere2DWindShear
