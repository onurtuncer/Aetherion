# Steady wind, Dryden turbulence, non-standard atmosphere — DONE (v0.16.0)

Plant-side half of Hemerion's sensor-realism plan (`C:\dev\Hemerion\TODO.md`,
"Sensor realism before the EKF is judged"). Drafted 2026-09-24 against
v0.15.0; implemented, tested and documented the same day on
`feat/wind-turbulence-atmosphere`; version bumped to **v0.16.0**.

Everything is opt-in and defaults to the calm, standard-day plant. Scenarios
11, 12 and 13.1 re-run after the change are **byte-identical** to the runs
made with the pre-change binaries (30 s each, `F16SteadyFlight`,
`F16SupersonicTrim`, `F16AltitudeChange`), which is the proof that the
defaults change nothing.

## What shipped

Library (`Aetherion/`):

- `Environment/Atmosphere.h`: `AtmosphereOffsets{deltaT_K, deltaP_sl_Pa}` and
  `US1976Atmosphere(alt, offsets)`. Hydrostatic re-integration of the layer
  base pressures from the shifted sea-level pressure through the shifted
  temperature profile; the published table is scaled by the ratio
  with/without offsets so zero offsets reproduce it bit for bit. One-entry
  memo so a non-standard day is as cheap as the table inside Newton.
- `Environment/DrydenTurbulence.h`: `GustState`, `DrydenParameters`,
  `DrydenLowAltitudeParameters(h, W20)` (MIL-F-8785C low-altitude rules) and
  `DrydenTurbulence` (8 states: u; v + r lag; w + q lag; p). Exact ZOH
  discretisation by Van Loan with a hand-written matrix exponential (Eigen's
  unsupported module is not in the tree), per-channel cache keyed on
  (sigma, L, V, corner, dt), Box-Muller on `mt19937_64` for cross-platform
  determinism, `states()/setStates()` for FMU checkpoints.
- `FlightDynamics/Policies/AirRelative.h`: `AirRelativeVelocity_B` and
  `AirRelativeRates_B`, the one place the ECI state becomes what the air sees.
  Used by `F16AeroPolicy`, `F16PropPolicy` and `RocketAeroPolicy`.
- `F16AeroPolicy`, `F16PropPolicy`: `setWindECEF`, `setGust`,
  `setAtmosphereOffsets` (+ getters); `F16AeroPolicy::airRelativeVelocity_B`
  / `airRelativeRates_B` / `atmosphere(alt)` public for whoever reports air
  data. `RocketAeroPolicy`: wind and offsets (no gust).
- `TrimSolver::setAtmosphereOffsets`.
- `MakeSnapshot1` (two-policy overload) and `MakeSnapshot2` read offsets,
  wind and gust from the aero policy: TAS, Mach, qbar and the atmosphere
  fields are the ones the forces used.
- `TwoStageRocketSimulator::aero()` accessor. `RigidBody/Config.h` stale
  comment fixed (the `wind`/`windShear` fields are parsed from JSON; kept).

FMUs:

- `F16Plant`: parameters `wind.{north,east,down}_mps`,
  `turb.sigma_{u,v,w}_mps`, `turb.L_{u,v,w}_m`, `turb.seed`, `atm.deltaT_K`,
  `atm.deltaP_sl_Pa`; outputs `out.wind_{north,east,down}_m_s` (steady +
  gust, NED), `out.gust_{u,v,w}_m_s`, `out.gust_{p,q,r}_rad_s`. Initial
  ground velocity = trim airspeed along heading + wind; the trim is
  air-relative and untouched. Turbulence stepped once per integrator
  sub-step at the current airspeed, warns once if the step varies. The eight
  filter states are in the saved FMU state (the noise stream is not
  rewound). The duplicate alpha/beta formula in `populateOutputCache` is
  gone; it calls the policy.
- `TwoStageRocket`: `wind.*`, `atm.*`.

Tests (all green, full suite 622/622 in Release, 211 s):

- `tests/Environment/test_atmosphere_offsets.cpp` (in `test_atmosphere`):
  bit-identity at zero offsets; ISA + 15 K → rho 1.1644; −1000 Pa → 83.6 m
  pressure-altitude error; dp/dh = −rho g0 (Re/(Re+h))² to 1e-7 for five
  offset pairs at eight altitudes; continuity across layer bases; hot day
  thinner below ~5 km and denser above (that is the physics: a warmer column
  decays more slowly); memo correctness; AD.
- `tests/Environment/test_DrydenTurbulence.cpp` (`test_dryden_turbulence`):
  variance of u, v, w = sigma² and of p = ∫Phi_p to 4 % over 2e5 s;
  averaged periodogram vs Phi_u and Phi_w at 0.15, 0.38, 1.0, 3.0 rad/s to
  15 % including the (1 + 3x²) tail; dt = 0.01 vs 0.04 agree; seed
  determinism; state round-trip; low-altitude rules.
- `tests/FlightDynamics/test_F16Environment.cpp` (`test_f16_environment`):
  default environment bit-identical; steady wind = ground-velocity shift and
  nothing else, at three winds and three Earth rotation angles; linear gust =
  opposite body-velocity shift; rotational gust signs (p, q, r all produce the
  restoring-sense moment); hot day scales forces by the density ratio
  (0.954 at 10 013 ft, +20 K) and trims at higher alpha (2.83° vs 2.64°);
  the engine sees the same air as the airframe in a tailwind and on a hot
  day; snapshot air data equals the policy's; AD evaluation equals double.
- FMU-level check under fmpy 0.3.32 (scratch script, not in ctest): calm
  holds altitude to 1 cm in 60 s; 10 m/s east wind leaves TAS and alpha
  unchanged, shifts the ground track 599 m east in 60 s with altitude held to 0.3 m, reports
  `out.wind_east_m_s = 10`; light turbulence gives gust_w std 1.3 m/s, finite
  outputs, seed-reproducible bit for bit; +20 K gives `out.T_K` +20.0 and
  density ratio 0.954 with the aircraft re-trimmed at higher alpha and
  altitude held to 1 cm.

Docs: `doc/aero_wind_policies.rst` new section "Environment on the Vehicle
Policies" (formulas, Dryden spectra, sign convention, discretisation,
parameters, atmosphere re-integration, the two headline numbers) and the
composition table; `doc/fmus.rst` parameter/output tables for both FMUs and
the version note; `doc/index.rst` version line; README bullets. Sphinx builds
with the same seven pre-existing errors (all in `examples.rst`), none new.

## Deviations from the plan, and why

- **Runtime setters instead of a `Wind` template parameter.** `F16AeroPolicy`
  is a concrete type in the `F16VF` alias used by every example and FMU;
  a template would have changed that type everywhere. Setters between steps
  are what the control surfaces already do, and the FMU needs exactly that.
- **No `turb.preset` FMU parameter.** The high-altitude intensities in
  MIL-F-8785C come from a probability-of-exceedance chart (Figure 7) that I
  will not transcribe from memory. The low-altitude rules are formulaic and
  live in `DrydenLowAltitudeParameters`; above 2000 ft the user sets the six
  numbers with the 533.4 m default scale lengths.
- **The propulsion policy needed the environment too.** Not in the plan.
  `F16PropPolicy` formed Mach from the ground-relative velocity and the
  standard speed of sound; in a 10 m/s tailwind the FMU descended 27 m in
  60 s (t³ growth, antisymmetric in head/tail wind, independent of crab),
  and a hot day drifted similarly. Found by the FMU-level check, fixed by
  the shared `AirRelative.h` and the same three setters on the propulsion
  policy; the FMU sets both policies alike. Test "the engine flies in the
  same air as the airframe" pins it.
- **Snapshot TAS.** `MakeSnapshot1` reports |v_NED| as TAS. Rather than
  switching it to the policy's body-frame vector (which would break
  bit-identity through rounding), it subtracts the wind rotated to NED only
  when the wind is non-zero, so calm stays bit-identical and windy is
  air-relative.
- **Rocket: wind and offsets, no gust.** Dryden is an aircraft model (it
  needs a wingspan and a mean airspeed); the rocket policy and FMU take the
  steady wind and the atmosphere offsets only.

## Still open

- Hot-day thrust: the engine deck is a standard-day table on altitude and
  Mach; the hot day changes only the Mach it is looked up at. Documented.
- The standalone F-16 example executables have no `--wind`/`--turbulence`
  flags; the FMU is the delivery path Hemerion uses. Add flags if a
  standalone turbulent scenario is ever wanted.
- Turbulence + `setFmuState`: filter states restore, the noise stream does
  not, so a restored run diverges from the original realisation after the
  first turbulent step. Documented in the FMU header.
- `RigidBody::Config::wind` / `windShear` are parsed from JSON but the F-16
  FMU takes its wind from FMI parameters, not from a Config file. Left as is.

Build note: Ninja did not see the header edits even with `VSLANG=1033`; the
final verification was done on a `--clean-first` build.
