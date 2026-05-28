// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// RocketStageModel.h
//
// Manages the multi-stage propulsion and variable-inertia state for the
// NASA TM-2015-218675 Scenario 17 two-stage rocket.
//
// This is the key abstraction that the framework was missing for multi-stage
// flight vehicles:
//
//   - Wraps the two DAVE-ML engines (twostage_prop.dml, twostage_inertia.dml).
//   - Tracks cumulative fuel consumed per stage (stg1FuelUsed_kg,
//     stg2FuelUsed_kg) and the binary staged flag.
//   - Provides propulsion() → {thrust_N, mdot_kgs} for the current firing phase.
//   - Provides inertialParameters() → RigidBody::InertialParameters for the
//     current fuel / staging state (XMASS, XIXX, XIYY, DXCG etc.).
//   - advance(dt, mdot) advances fuel state and detects the S1-burnout staging
//     event, returning true on the step in which staging occurs.
//
// The simulator calls this model once before each integration step (ZOH) to
// update the VectorField's spatial inertia matrix M / M_inv and the
// AxialThrustPolicy.thrust_N / LinearBurnPolicy.mdot_kgs members.  On a
// staging event the simulator additionally subtracts kStg1DryMass_kg from the
// ODE state mass and rebuilds M.
//
// DAVE-ML variable IDs used:
//   Prop inputs : stg1firing, stg2firing (nd, >0 = firing)
//   Prop outputs: thrust [N], mdot [kg/s]
//
//   Inertia inputs : stagedFlag (nd), stg1fuelUsed [kg], stg2fuelUsed [kg]
//   Inertia outputs: XMASS [kg], XIXX [kg·m²], XIYY [kg·m²], XIZZ [kg·m²],
//                    XIZX, XIXY, XIYZ [kg·m²] (all zero for this vehicle),
//                    DXCG [m, sign=+FWD] (CG forward of MRC → xbar_m = DXCG)
//
// Units: the rocket DML files use SI units throughout (m, kg, kgm2, N, kg/s).
// evaluateRaw() returns values without conversion, which is correct here.
// ------------------------------------------------------------------------------

#pragma once

#include <Aetherion/RigidBody/InertialParameters.h>
#include <Aetherion/Serialization/DAVEML/DAVEMLAeroModel.h>

#include <memory>
#include <string>

namespace Aetherion::Examples::TwoStageRocket {

// ─────────────────────────────────────────────────────────────────────────────
// RocketFuelState
// ─────────────────────────────────────────────────────────────────────────────

/// @brief Cumulative fuel-consumption and staging state for the two-stage rocket.
struct RocketFuelState {
    double stg1FuelUsed_kg = 0.0; ///< Stage-1 propellant consumed so far [kg].
    double stg2FuelUsed_kg = 0.0; ///< Stage-2 propellant consumed so far [kg].
    bool   staged          = false; ///< True once S1 dry stage has been jettisoned.
};

// ─────────────────────────────────────────────────────────────────────────────
// RocketPropulsionResult
// ─────────────────────────────────────────────────────────────────────────────

/// @brief Instantaneous propulsion output for one integration step.
struct RocketPropulsionResult {
    double thrust_N  = 0.0; ///< Total axial thrust [N, body +x].
    double mdot_kgs  = 0.0; ///< Propellant consumption rate [kg/s, positive = burn].
};

// ─────────────────────────────────────────────────────────────────────────────
// RocketStageModel
// ─────────────────────────────────────────────────────────────────────────────

/// @brief Multi-stage rocket propulsion and inertia manager.
///
/// Owns the DAVE-ML evaluation engines for @c twostage_prop.dml and
/// @c twostage_inertia.dml, tracks the cumulative fuel state, and exposes
/// the per-step propulsion result and inertial parameters that the simulator
/// uses to update the VectorField between integration steps.
///
/// **Staging event**: when @c advance() detects that S1 propellant is exhausted,
/// it sets @c staged = true and returns @c true.  The caller (simulator) must
/// then subtract @c kStg1DryMass_kg from the ODE state mass to model the
/// stage-separation mass drop.
class RocketStageModel {
public:
    // ── Rocket constants (from twostage_inertia.dml) ──────────────────────────
    static constexpr double kStg1MaxFuel_kg  = 180000.0; ///< S1 propellant capacity [kg].
    static constexpr double kStg2MaxFuel_kg  =  80000.0; ///< S2 propellant capacity [kg].
    /// Dry mass of stage-1 hardware jettisoned at staging [kg].
    /// Derived from stg1burnoutMass(134 000) − stg2igniteMass(99 000).
    static constexpr double kStg1DryMass_kg  =  35000.0;

    // ── Construction ──────────────────────────────────────────────────────────

    /// @brief Construct from pre-loaded DAVE-ML engine instances.
    /// @param inertiaDml  Parsed @c twostage_inertia.dml engine.
    /// @param propDml     Parsed @c twostage_prop.dml engine.
    explicit RocketStageModel(
        std::shared_ptr<Serialization::DAVEMLAeroModel> inertiaDml,
        std::shared_ptr<Serialization::DAVEMLAeroModel> propDml);

    // ── Per-step queries ──────────────────────────────────────────────────────

    /// @brief Evaluate the propulsion DML at the current firing state.
    /// @return Thrust [N] and propellant flow rate [kg/s] for this step.
    [[nodiscard]] RocketPropulsionResult propulsion() const;

    /// @brief Evaluate the inertia DML at the current fuel state.
    /// @return Mass properties referenced to the moment reference centre (MRC).
    ///         xbar_m = DXCG [m, +FWD] encodes the CG–MRC offset.
    [[nodiscard]] RigidBody::InertialParameters inertialParameters() const;

    // ── State advance ─────────────────────────────────────────────────────────

    /// @brief Advance the cumulative fuel state by one step.
    ///
    /// @param dt_s      Step duration [s].
    /// @param mdot_kgs  Propellant consumption rate for this step [kg/s, positive = burn].
    /// @return @c true if a staging event occurred during this advance (S1 exhausted).
    ///         The caller must apply the @c kStg1DryMass_kg mass drop to the ODE state.
    bool advance(double dt_s, double mdot_kgs);

    // ── Accessors ─────────────────────────────────────────────────────────────

    [[nodiscard]] const RocketFuelState& fuelState() const noexcept { return m_fuel; }

private:
    std::shared_ptr<Serialization::DAVEMLAeroModel> m_inertiaDml;
    std::shared_ptr<Serialization::DAVEMLAeroModel> m_propDml;
    RocketFuelState m_fuel;
};

} // namespace Aetherion::Examples::TwoStageRocket
