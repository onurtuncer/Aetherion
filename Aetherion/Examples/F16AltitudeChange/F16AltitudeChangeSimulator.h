// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// F16AltitudeChangeSimulator.h
//
// Six-DoF closed-loop simulator for NASA TM-2015-218675 Atmospheric Scenario 13.1
// (F-16 subsonic altitude change).
//
// Architecture:
//   Identical VectorField to Case 11 (J2 gravity + DAVE-ML aero + DAVE-ML prop).
//   At each step() call the simulator:
//     1. Extracts state feedback (Euler angles, body rates, alpha, beta, KEAS, alt).
//     2. Evaluates the DML-driven LQR control law via DAVEMLControlModel.
//     3. Updates F16AeroPolicy.{el,ail,rdr}_deg and F16PropPolicy.pwr_pct.
//     4. Advances the ODE by h seconds with the freshly updated surface commands.
//
// This is a zero-order hold (ZOH) discrete controller at the integration step
// rate, matching the NASA reference implementation.
// ------------------------------------------------------------------------------

#pragma once

#include <Aetherion/Examples/F16SteadyFlight/F16Types.h>
#include <Aetherion/Simulation/ISimulator.h>
#include <Aetherion/Simulation/Snapshot1.h>
#include <Aetherion/Simulation/MakeSnapshot1.h>
#include <Aetherion/Serialization/DAVEML/DAVEMLControlModel.h>
#include <Aetherion/FlightDynamics/Trim/TrimSolver.h>
#include <Aetherion/RigidBody/InertialParameters.h>
#include <Aetherion/ODE/RKMK/Core/NewtonOptions.h>
#include <Aetherion/Coordinate/InertialToLocal.h>
#include <Aetherion/Environment/Atmosphere.h>
#include <Aetherion/Environment/WGS84.h>

#include <memory>
#include <cmath>
#include <numbers>

namespace Aetherion::Examples::F16AltitudeChange {

using F16VF = F16SteadyFlight::F16VF;

/// @brief Autopilot commands for the F16AltitudeChangeSimulator (fixed for the whole run).
///
/// Defined at namespace scope so that its in-class default member initializers
/// are complete before any enclosing class uses it as a default argument — a
/// requirement that GCC enforces but MSVC relaxes.
struct F16AltitudeChangeCmds {
    double altCmd_ft         { 10113.0 };             ///< Commanded altitude [ft]
    double keasCmd_kt        { 287.8088596053291 };   ///< Commanded KEAS [kt]
    double baseChiCmd_deg    {  45.0  };              ///< Commanded course [deg, +CW from N]
    double latOffset_ft      {   0.0  };              ///< Lateral offset from course [ft]
    /// Time [s] at which altCmd_ft is applied.  Before this time the
    /// controller receives initialAltCmd_ft instead, matching the NASA
    /// reference implementation that holds the trim altitude until the step.
    /// Default 0.0 = apply immediately (correct for Scenarios 13.2–13.4).
    double altStepTime_s     {   0.0  };
    double initialAltCmd_ft  { 10013.0 };             ///< Altitude held before altStepTime_s [ft]

    /// Time [s] at which keasCmd_kt is applied.  Before this time the
    /// controller receives initialKeasCmd_kt instead, matching the NASA
    /// reference implementation that holds the trim KEAS until the step.
    /// Default 0.0 = apply immediately (correct for Scenarios 13.1, 13.3–13.4).
    double keasStepTime_s    {   0.0  };
    double initialKeasCmd_kt { 287.0  };              ///< KEAS held before keasStepTime_s [kt]

    /// Time [s] at which baseChiCmd_deg is applied.  Before this time the
    /// controller receives initialChiCmd_deg instead, matching the NASA
    /// reference implementation that holds the trim heading until the step.
    /// Default 0.0 = apply immediately (correct for Scenarios 13.1/13.2).
    double chiStepTime_s     {   0.0  };
    double initialChiCmd_deg {  45.0  };              ///< Course held before chiStepTime_s [deg]

    /// Lateral side-step parameters (Scenario 13.4).
    /// When latStepOffset_ft != 0, the simulator computes latOffset dynamically
    /// from the aircraft's position relative to the original courseline, applying
    /// the commanded offset at latStepTime_s.  Set both to 0 (default) to fall
    /// back to the fixed latOffset_ft value (correct for Scenarios 13.1–13.3).
    double latStepTime_s     {   0.0  };              ///< When to apply lat step [s]
    double latStepOffset_ft  {   0.0  };              ///< Commanded lateral offset right of course [ft]
    double coursePsi_deg     {  45.0  };              ///< Reference course direction [deg CW from N]
};

class F16AltitudeChangeSimulator
    : public Simulation::ISimulator<F16VF, Simulation::Snapshot1>
{
public:
    static constexpr double kOmegaEarth_rad_s = Environment::WGS84::kRotationRate_rad_s;
    static constexpr double kFt_m             = FlightDynamics::TrimSolver::kFt_m;
    // Sea-level air density for KEAS computation [kg/m³]
    static constexpr double kRhoSL_kg_m3      = 1.225;
    // kt → m/s
    static constexpr double kKt_mps           = 0.5144444;

    /// Convenience alias so callers can write F16AltitudeChangeSimulator::AutopilotCmds.
    using AutopilotCmds = F16AltitudeChangeCmds;

    explicit F16AltitudeChangeSimulator(
        const RigidBody::InertialParameters&                      ip,
        const FlightDynamics::TrimPoint&                          trim,
        std::shared_ptr<const Serialization::DAVEMLAeroModel>     aero_model,
        std::shared_ptr<const Serialization::DAVEMLPropModel>     prop_model,
        std::shared_ptr<const Serialization::DAVEMLControlModel>  ctrl_model,
        RigidBody::StateD                                         x0,
        double                                                    theta0,
        AutopilotCmds                                             cmds = {},
        double                                                    xcg_from_ac_m = 0.0,
        ODE::RKMK::Core::NewtonOptions                            opt = {})
        : ISimulator<F16VF, Simulation::Snapshot1>(
            F16VF(
                ip,
                FlightDynamics::J2GravityPolicy{},
                FlightDynamics::F16AeroPolicy(aero_model, trim.el_deg, 0.0, 0.0,
                                              xcg_from_ac_m),
                FlightDynamics::F16PropPolicy(prop_model, trim.pwr_pct)),
            std::move(x0), opt)
        , m_theta0(theta0)
        , m_ctrl(std::move(ctrl_model))
        , m_cmds(cmds)
        , m_trimPwr(trim.pwr_pct)
        , m_trimEl(trim.el_deg)
    {
        // Store initial geodetic position for dynamic latOffset computation (Scenario 13.4)
        namespace Coord = Aetherion::Coordinate;
        const auto& p0 = state().g.p;
        Coord::Vec3<double> r0_ecef = Coord::ECIToECEF(
            Coord::Vec3<double>{ p0.x(), p0.y(), p0.z() }, theta0);
        double alt0_m{};
        Coord::ECEFToGeodeticWGS84(r0_ecef, m_lat0_rad, m_lon0_rad, alt0_m);
    }

    // ── Closed-loop step ──────────────────────────────────────────────────────

    /// @brief Evaluate control law → update VF policies → advance ODE by h.
    [[nodiscard]] StepResult step(double h)
    {
        if (m_ctrl) {
            const auto fb  = extractFeedback();
            const auto out = m_ctrl->evaluate(fb);
            applyControls(out);
        }
        return ISimulator<F16VF, Simulation::Snapshot1>::step(h);
    }

    // ── Snapshot ──────────────────────────────────────────────────────────────

    [[nodiscard]]
    Simulation::Snapshot1 snapshot() const noexcept override
    {
        return Simulation::MakeSnapshot1(
            time(), state(), currentTheta(),
            vectorField().gravity, vectorField().aero);
    }

    [[nodiscard]] double currentTheta() const noexcept
    {
        return m_theta0 + kOmegaEarth_rad_s * time();
    }

protected:
    void validate() const override {}

private:
    // ── Feedback extraction ───────────────────────────────────────────────────

    [[nodiscard]] Serialization::DAVEMLControlModel::Inputs extractFeedback() const noexcept;

    // ── Control application ───────────────────────────────────────────────────

    void applyControls(const Serialization::DAVEMLControlModel::Outputs& out) noexcept
    {
        auto& vf = mutableVectorField();
        vf.aero.el_deg    = out.el_deg;
        vf.aero.ail_deg   = out.ail_deg;
        vf.aero.rdr_deg   = out.rdr_deg;
        vf.thrust.pwr_pct = out.pwr_pct;
    }

    double m_theta0;
    std::shared_ptr<const Serialization::DAVEMLControlModel> m_ctrl;
    AutopilotCmds m_cmds;
    double m_trimPwr;
    double m_trimEl;
    double m_lat0_rad{ 0.0 };   // initial geodetic latitude [rad]  (for dynamic latOffset)
    double m_lon0_rad{ 0.0 };   // initial geodetic longitude [rad] (for dynamic latOffset)
};

// ── extractFeedback — inline implementation ───────────────────────────────────
//
// Replicates the state extraction done in MakeSnapshot1 and F16AeroPolicy so
// that the controller receives the same physical quantities the integrator uses.

inline Serialization::DAVEMLControlModel::Inputs
F16AltitudeChangeSimulator::extractFeedback() const noexcept
{
    namespace Coord = Aetherion::Coordinate;
    namespace Env   = Aetherion::Environment;
    using Vec3d     = Eigen::Vector3d;

    const auto& s      = state();
    const double theta = currentTheta();   // Earth Rotation Angle

    // ── Geodetic position ─────────────────────────────────────────────────────
    Coord::Vec3<double> r_ecef_arr = Coord::ECIToECEF(
        Coord::Vec3<double>{ s.g.p.x(), s.g.p.y(), s.g.p.z() }, theta);

    double lat_rad{}, lon_rad{}, alt_m{};
    Coord::ECEFToGeodeticWGS84(r_ecef_arr, lat_rad, lon_rad, alt_m);

    // ── Atmosphere ────────────────────────────────────────────────────────────
    const auto atm = Env::US1976Atmosphere(alt_m);

    // ── Atmosphere-relative body velocity (same calc as F16AeroPolicy) ────────
    constexpr double kOmE = Environment::WGS84::kRotationRate_rad_s;
    const Vec3d omega_E(0.0, 0.0, kOmE);
    const Vec3d v_surface = s.g.R.transpose() * omega_E.cross(s.g.p);
    const Vec3d v_rel     = s.nu_B.tail<3>() - v_surface;

    const double u = v_rel(0);
    const double v = v_rel(1);
    const double w = v_rel(2);
    const double vt_mps = std::sqrt(v_rel.squaredNorm() + 1.0e-30);

    const double alpha_deg = std::atan2(w, u + 1.0e-12) * (180.0 / std::numbers::pi);
    const double beta_deg  = std::asin(std::clamp(v / vt_mps, -1.0, 1.0))
                             * (180.0 / std::numbers::pi);

    // ── Euler angles from body-to-NED rotation ────────────────────────────────
    const Coord::Mat3<double> R_IN_arr =
        Coord::NEDToInertialRotationMatrix(lat_rad, lon_rad, theta);
    Eigen::Matrix3d R_IN;
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            R_IN(r, c) = R_IN_arr[3 * r + c];

    const Eigen::Matrix3d R_NB = R_IN.transpose() * s.g.R;
    const double theta_deg = std::asin(std::clamp(-R_NB(2, 0), -1.0, 1.0))
                             * (180.0 / std::numbers::pi);
    const double phi_deg   = std::atan2(R_NB(2, 1), R_NB(2, 2))
                             * (180.0 / std::numbers::pi);
    const double psi_deg   = std::atan2(R_NB(1, 0), R_NB(0, 0))
                             * (180.0 / std::numbers::pi);

    // ── Body angular rates (ECI-relative, in body frame) ─────────────────────
    const double p_rad_s = s.nu_B(0);
    const double q_rad_s = s.nu_B(1);
    const double r_rad_s = s.nu_B(2);

    // ── KEAS [kt] ─────────────────────────────────────────────────────────────
    const double tas_kt  = vt_mps / kKt_mps;
    const double keas_kt = tas_kt * std::sqrt(atm.rho / kRhoSL_kg_m3);

    // ── Assemble Inputs ───────────────────────────────────────────────────────
    Serialization::DAVEMLControlModel::Inputs fb;
    // Pilot inputs: all zero (autopilot active)
    fb.throttle_frac  = 0.0;
    fb.longStk_frac   = 0.0;
    fb.latStk_frac    = 0.0;
    fb.pedal_frac     = 0.0;
    // Modes: both on
    fb.sasOn          = 1.0;
    fb.apOn           = 1.0;
    // Autopilot commands — alt step is held at initialAltCmd_ft until altStepTime_s
    fb.altCmd_ft      = (time() >= m_cmds.altStepTime_s)
                        ? m_cmds.altCmd_ft
                        : m_cmds.initialAltCmd_ft;
    fb.keasCmd_kt     = (time() >= m_cmds.keasStepTime_s)
                        ? m_cmds.keasCmd_kt
                        : m_cmds.initialKeasCmd_kt;
    // chi step is held at initialChiCmd_deg until chiStepTime_s
    fb.baseChiCmd_deg = (time() >= m_cmds.chiStepTime_s)
                        ? m_cmds.baseChiCmd_deg
                        : m_cmds.initialChiCmd_deg;
    // Dynamic lateral offset (Scenario 13.4): compute aircraft's lateral deviation
    // from the original courseline; subtract the commanded step offset applied
    // at latStepTime_s.  Fall back to fixed latOffset_ft when no step is set.
    if (m_cmds.latStepOffset_ft != 0.0) {
        constexpr double kRE_m = 6'371'000.0;
        const double dp_N_m = (lat_rad  - m_lat0_rad) * kRE_m;
        const double dp_E_m = (lon_rad  - m_lon0_rad) * kRE_m * std::cos(m_lat0_rad);
        const double psi0   = m_cmds.coursePsi_deg * (std::numbers::pi / 180.0);
        const double lat_dev_ft = (-dp_N_m * std::sin(psi0) + dp_E_m * std::cos(psi0))
                                   / kFt_m;
        const double step_ft = (time() >= m_cmds.latStepTime_s)
                               ? m_cmds.latStepOffset_ft : 0.0;
        fb.latOffset_ft = lat_dev_ft - step_ft;
    } else {
        fb.latOffset_ft = m_cmds.latOffset_ft;
    }
    // Sensor feedbacks
    fb.altMsl_ft      = alt_m / kFt_m;
    fb.Vequiv_kt      = keas_kt;
    fb.alpha_deg      = alpha_deg;
    fb.beta_deg       = beta_deg;
    fb.phi_deg        = phi_deg;
    fb.theta_deg      = theta_deg;
    fb.psi_deg        = psi_deg;
    fb.pb_rad_s       = p_rad_s;
    fb.qb_rad_s       = q_rad_s;
    fb.rb_rad_s       = r_rad_s;
    // Trim values use struct defaults (match the DML initialValue constants)

    return fb;
}

} // namespace Aetherion::Examples::F16AltitudeChange
