// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// DraglessSphereApplication.h
//
// Simulation::Application subclass for the NASA TM-2015-218675 Atmospheric
// Scenario 1 (dragless sphere, J2 gravitation).
//
// Design
// ──────
// Composition over inheritance for the simulator:
//   - DraglessSphereApplication IS-A Simulation::Application  (inherits)
//   - DraglessSphereApplication HAS-A DraglessSphereSimulator (owns via unique_ptr)
//
// Run loop (run() override):
//   1. Log startup info (config, initial state)
//   2. Load RigidBody::Config from --inputFileName
//   3. Build initial StateD via BuildInitialStateVector
//   4. Construct DraglessSphereSimulator
//   5. Open output CSV at --outputFileName, write Snapshot1 header
//   6. Log initial snapshot (t = startTime)
//   7. Time-step loop from startTime to endTime using timeStep
//   8. Write Snapshot1 row each step; log progress at INFO level
//   9. Log final state summary
// ------------------------------------------------------------------------------

#pragma once

#include <Aetherion/Simulation/Application.h>
#include <Aetherion/Simulation/Log.h>
#include <Aetherion/Simulation/Snapshot1.h>
#include <Aetherion/Simulation/MakeSnapshot1.h>
#include <Aetherion/Serialization/LoadConfig.h>
#include <Aetherion/FlightDynamics/BuildInitialStateVector.h>
#include <Aetherion/Examples/DraglessSphere/DraglessSphereSimulator.h>

#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>

namespace Aetherion::Examples::DraglessSphere {

    class DraglessSphereApplication : public Simulation::Application
    {
    public:
        // -----------------------------------------------------------------------
        // Construction: forward argc/argv to Application (parses CLI flags).
        // The simulator is created lazily inside run().
        // -----------------------------------------------------------------------
        explicit DraglessSphereApplication(int argc, char* argv[])
            : Application(argc, argv)
        {
        }

        // -----------------------------------------------------------------------
        // run() — the full NASA Atmos-01 simulation loop.
        // -----------------------------------------------------------------------
        void run() const override
        {
            // ── 0. Initialise logging ─────────────────────────────────────────
            Simulation::Log::Init();

            const auto& cfg_sim = getConfig();   // Simulation::Config (CLI)

            AE_CORE_INFO("=======================================================");
            AE_CORE_INFO("Aetherion — NASA TM-2015-218675 Atmospheric Scenario 1");
            AE_CORE_INFO("Dragless sphere · J2 gravitation · US 1976 atmosphere");
            AE_CORE_INFO("=======================================================");

            AE_CORE_INFO("Simulation parameters:");
            AE_CORE_INFO("  startTime      = {:.6f} s", cfg_sim.startTime);
            AE_CORE_INFO("  endTime        = {:.6f} s", cfg_sim.endTime);
            AE_CORE_INFO("  timeStep       = {:.6f} s", cfg_sim.timeStep);
            AE_CORE_INFO("  inputFileName  = {}", cfg_sim.inputFileName);
            AE_CORE_INFO("  outputFileName = {}", cfg_sim.outputFileName);

            // ── 1. Load vehicle config from JSON ─────────────────────────────
            if (cfg_sim.inputFileName.empty())
                throw std::runtime_error(
                    "No --inputFileName specified. "
                    "Provide the path to the vehicle JSON config.");

            AE_CORE_INFO("Loading vehicle config from '{}'", cfg_sim.inputFileName);
            const RigidBody::Config rb_cfg =
                Serialization::LoadConfig(cfg_sim.inputFileName);

            AE_CORE_DEBUG("Vehicle config loaded:");
            AE_CORE_DEBUG("  lat={:.4f} deg  lon={:.4f} deg  alt={:.2f} m",
                rb_cfg.pose.lat_deg, rb_cfg.pose.lon_deg, rb_cfg.pose.alt_m);
            AE_CORE_DEBUG("  azimuth={:.2f} deg  zenith={:.2f} deg  roll={:.2f} deg",
                rb_cfg.pose.azimuth_deg, rb_cfg.pose.zenith_deg, rb_cfg.pose.roll_deg);
            AE_CORE_DEBUG("  v_NED  = [{:.4f}, {:.4f}, {:.4f}] m/s",
                rb_cfg.velocityNED.north_mps,
                rb_cfg.velocityNED.east_mps,
                rb_cfg.velocityNED.down_mps);
            AE_CORE_DEBUG("  omega_B = [{:.6e}, {:.6e}, {:.6e}] rad/s",
                rb_cfg.bodyRates.roll_rad_s,
                rb_cfg.bodyRates.pitch_rad_s,
                rb_cfg.bodyRates.yaw_rad_s);
            AE_CORE_DEBUG("  mass   = {:.4f} kg", rb_cfg.inertialParameters.mass_kg);

            // ── 2. Compute initial Earth Rotation Angle ───────────────────────
            //   theta0 = omega_E * t0   (simple linear ERA from J2000-like epoch)
            const double theta0 =
                DraglessSphereSimulator::kOmegaEarth_rad_s * cfg_sim.startTime;

            AE_CORE_DEBUG("Initial ERA theta0 = {:.10f} rad  ({:.6f} deg)",
                theta0, theta0 * (180.0 / 3.14159265358979323846));

            // ── 3. Build initial StateD ───────────────────────────────────────
            AE_CORE_INFO("Building initial state vector (ECI frame)...");
            const auto x0 =
                FlightDynamics::BuildInitialStateVector(rb_cfg, theta0);

            // Reconstruct StateD from the flat vector
            RigidBody::StateD initial_state;
            {
                using Layout = RigidBody::StateLayout;
                // Position
                initial_state.g.p =
                    Eigen::Vector3d(x0[Layout::IDX_P], x0[Layout::IDX_P + 1], x0[Layout::IDX_P + 2]);
                // Attitude quaternion (w, x, y, z) → Eigen::Matrix3d rotation
                const Eigen::Quaterniond q(
                    x0[Layout::IDX_Q],      // w
                    x0[Layout::IDX_Q + 1],  // x
                    x0[Layout::IDX_Q + 2],  // y
                    x0[Layout::IDX_Q + 3]); // z
                initial_state.g.R = q.normalized().toRotationMatrix();
                // Body twist [omega_B; v_B]
                initial_state.nu_B <<
                    x0[Layout::IDX_W], x0[Layout::IDX_W + 1], x0[Layout::IDX_W + 2],
                    x0[Layout::IDX_V], x0[Layout::IDX_V + 1], x0[Layout::IDX_V + 2];
                // Mass
                initial_state.m = x0[Layout::IDX_M];
            }

			//TODO [Onur] write proper output formatters in Log.h and use those instead of raw {:.3f} etc. formatters

            AE_CORE_INFO("Initial ECI position  = [{:.3f}, {:.3f}, {:.3f}] m",
                initial_state.g.p.x(), initial_state.g.p.y(), initial_state.g.p.z());
            AE_CORE_INFO("Initial ECI position magnitude = {:.3f} m",
                initial_state.g.p.norm());
            AE_CORE_INFO("Initial body twist nu_B = [{:.6e}, {:.6e}, {:.6e}, {:.6e}, {:.6e}, {:.6e}]",
                initial_state.nu_B(0), initial_state.nu_B(1), initial_state.nu_B(2),
                initial_state.nu_B(3), initial_state.nu_B(4), initial_state.nu_B(5));

            // ── 4. Construct simulator ────────────────────────────────────────
            AE_CORE_INFO("Constructing DraglessSphereSimulator (J2 gravity)...");
            DraglessSphereSimulator simulator(
                rb_cfg.inertialParameters,
                initial_state,
                theta0);

            // ── 5. Open output CSV ────────────────────────────────────────────
            AE_CORE_INFO("Opening output file '{}'", cfg_sim.outputFileName);
            std::ofstream csv(cfg_sim.outputFileName);
            if (!csv.is_open())
                throw std::runtime_error(
                    "Cannot open output file: " + cfg_sim.outputFileName);

            Simulation::Snapshot1_WriteCsvHeader(csv);
            AE_CORE_DEBUG("CSV header written ({} columns).",
                Simulation::Snapshot1CsvTraits::kColumnCount);

            // ── 6. Write initial snapshot (t = startTime) ────────────────────
            {
                const auto snap0 = simulator.snapshot();
                Simulation::Snapshot1_WriteCsvRow(csv, snap0);
                AE_CORE_INFO("t={:.6f} s  alt={:.3f} m  |v_NED|={:.6f} m/s",
                    snap0.time,
                    snap0.altitudeMsl_m,
                    snap0.feVelocity_m_s.norm());
            }

            // ── 7. Time-step loop ─────────────────────────────────────────────
            const double dt = cfg_sim.timeStep;
            const double t_end = cfg_sim.endTime;

            AE_CORE_INFO("Starting integration: t_end={:.3f} s, dt={:.4f} s",
                t_end, dt);

            std::size_t step_count = 0;
            std::size_t failed_steps = 0;

            while (simulator.time() < t_end - 1e-12)
            {
                // Clamp last step so we never overshoot
                const double h = std::min(dt, t_end - simulator.time());

                const auto res = simulator.step(h);
                ++step_count;

                if (!res.converged)
                {
                    ++failed_steps;
                    AE_CORE_WARN(
                        "Step {} did not converge at t={:.6f} s  "
                        "(residual={:.3e}). Continuing.",
                        step_count, simulator.time(), res.residual_norm);
                }

                // Write snapshot every step
                const auto snap = simulator.snapshot();
                Simulation::Snapshot1_WriteCsvRow(csv, snap);

                // Log at INFO every 10 steps, TRACE every step
                AE_CORE_TRACE(
                    "  step {:4d}  t={:.6f} s  alt={:.4f} m  "
                    "v={:.6f} m/s  Mach={:.6f}",
                    step_count, snap.time,
                    snap.altitudeMsl_m,
                    snap.feVelocity_m_s.norm(),
                    snap.mach);

                if (step_count % 10 == 0 || simulator.time() >= t_end - 1e-12)
                {
                    AE_CORE_INFO(
                        "  t={:.4f} s  alt={:.3f} m  |v_NED|={:.4f} m/s  "
                        "lat={:.6f} deg  lon={:.6f} deg",
                        snap.time,
                        snap.altitudeMsl_m,
                        snap.feVelocity_m_s.norm(),
                        snap.latitude_rad * (180.0 / 3.14159265358979323846),
                        snap.longitude_rad * (180.0 / 3.14159265358979323846));
                }
            }

            csv.close();

            // ── 8. Final summary ──────────────────────────────────────────────
            const auto snap_final = simulator.snapshot();

            AE_CORE_INFO("=======================================================");
            AE_CORE_INFO("Simulation complete.");
            AE_CORE_INFO("  Total steps     : {}  (failed: {})", step_count, failed_steps);
            AE_CORE_INFO("  Final time      : {:.6f} s", snap_final.time);
            AE_CORE_INFO("  Final altitude  : {:.4f} m", snap_final.altitudeMsl_m);
            AE_CORE_INFO("  Final |v_NED|   : {:.6f} m/s", snap_final.feVelocity_m_s.norm());
            AE_CORE_INFO("  Final latitude  : {:.8f} deg",
                snap_final.latitude_rad * (180.0 / 3.14159265358979323846));
            AE_CORE_INFO("  Final longitude : {:.8f} deg",
                snap_final.longitude_rad * (180.0 / 3.14159265358979323846));
            AE_CORE_INFO("  Final Mach      : {:.6f}", snap_final.mach);
            AE_CORE_INFO("  Final q (body→ECI) : [{:.6f}, {:.6f}, {:.6f}, {:.6f}]",
                snap_final.q_body_to_eci.w(),
                snap_final.q_body_to_eci.x(),
                snap_final.q_body_to_eci.y(),
                snap_final.q_body_to_eci.z());
            AE_CORE_INFO("Output written to '{}'", cfg_sim.outputFileName);
            AE_CORE_INFO("=======================================================");
        }
    };

} // namespace Aetherion::Examples::DraglessSphere