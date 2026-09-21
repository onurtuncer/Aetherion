# TODO — resume here

Branch: **`fix/rkmk-left-trivialisation`** (2 commits, not yet pushed or merged)

## What happened

While filling in the convergence-study section of the EUCASS integrator paper,
the convergence test turned out to be degenerate: it used a torque-free
**symmetric** sphere, whose body twist is constant, so the exact flow is a
one-parameter subgroup that *any* RKMK method reproduces exactly at every step
size. Errors sat at ~1e-16 for both integrators at every `h` — no measurable
order.

Replacing it with a torque-free **asymmetric** body (nonlinear Euler equations,
so the twist actually varies) exposed a real bug: both integrators converged at
**order 2**, with errors agreeing to five significant figures — a signature that
the limiting factor was neither Butcher tableau.

Root cause: the kinematic ODE is left-trivialised (`g_dot = g * hat(xi)`), so
with `g(t) = g0 * Exp(u(t))` the stage correction is `u_dot = dexp_{-u}^{-1}(xi)`.
Every call site evaluated `dexp^-1` at `+eta` — the right-trivialised convention,
and the form the Bernoulli series is usually tabulated in. That flips the sign of
the `O(ad)` term and caps the method at order 2 for any tableau.

It was invisible to every existing test: the pose still stays exactly on SE(3),
Newton still converges, and constant-twist motion is still integrated exactly
(`ad_eta eta = 0` annihilates the bad term) — which is exactly what the old
convergence test exercised.

The same error was in `doc/rkmk.rst`, which is presumably where the
implementation convention came from.

## Done

- [x] Fix `dexp^-1` argument at all three call sites
      (`StageResidualIRKProductSE3.h`, `StageResidualRadauIIASE3.h`,
      `ExplicitRK4_RKMK_ProductSE3.h`).
      Radau IIA: order 2.00 -> **5.00**. Explicit RK4: 2.00 -> **3.91**.
- [x] Rewrite `tests/RigidBody/test_ConvergenceOrder.cpp`: asymmetric body,
      fine-step reference cross-validated between the two integrators, symmetric
      sphere retained as an *exactness* test with a comment on why it cannot
      measure order.
- [x] New `tests/RigidBody/test_ConstraintDrift.cpp`: SE(3) RKMK vs
      unconstrained quaternion RK4, with and without per-step renormalisation.
      Over 600 s: 4.4e-14 / 2.1e-3 / 6.9e-16.
- [x] Full test suite green in Release: 568/571.
      (The 3 failures are `*_ecos_ecos` FMI co-simulation segfaults, pre-existing
      and unrelated — confirm separately, see below.)
- [x] Re-ran 16 of 17 NASA scenarios in Release and re-measured everything.
- [x] `doc/rkmk.rst`: corrected derivation, added a warning admonition, fixed the
      SO(3) Jacobian labelling (the closed forms were right, the names were not),
      added `_rkmk_order_verification` section.
- [x] `doc/unified_notation.rst`: same `-eta` correction.
- [x] `doc/examples.rst`: added `_reference_run_spread` ("how to read the
      validation numbers") with the full table; corrected the stale per-scenario
      error claims for scenarios 1, 2, 3, 6, 7.
- [x] `papers/eucass/main.tex`: trivialisation section, real convergence and
      drift tables + figures, reference-spread validation table.
- [x] `papers/eucass-library/main.tex`: **new** library/marketing paper.
- [x] `plot_convergence.py` rewritten for the new CSV schema;
      `plot_constraint_drift.py` added. Both figures generated.
- [x] Sphinx builds with no new errors (7 before, 7 after — all pre-existing).

## Next — in priority order

### 1. Scenario 17 — run completed, eucass paper filled

The full 200 s run now completes. Launch it **detached** (`Start-Process`), not
through a tool-managed background job: those cap out at 10 minutes, which is
what truncated the earlier attempt at t = 31.4 s.

```
out\build\windows-release\src\Examples\TwoStageRocket\TwoStageRocket.exe \
    --outputFileName atmos17.csv --startTime 0 --endTime 200 \
    --timeStep 0.001 --writeInterval 100
```

`--endTime` must be 200: S2 ignition is computed as
`endTime - postBurnCoast - burnDuration`, so a different end time silently
moves the staging event and invalidates the comparison.

- [x] Recompute vs `Atmos_17_sim_04/05/06` — `scripts/compare_atmos17_errors.py`
      (committed, no longer a scratch script).
- [x] Scenario 17 error table in `papers/eucass/main.tex` filled, with figure
      `papers/eucass/scripts/plot_scenario17.py`.
- [x] Reconciled the comparison scripts. The disagreement was the down-range
      metric: Aetherion writes SI and the NASA files are imperial, and the
      earlier small-angle construction carried `(R_earth + h)` with h up to
      234 km, inflating range by 3.7 %. Now a haversine ground range on
      `R_earth`, stated explicitly in the script docstring.
- [x] `papers/eucass-library/main.tex`: Scenario 17 row, figure and prose filled;
      also fixed a literal U+0307 combining dot (`u̇` -> `\dot u`) that was a hard
      LaTeX error, and the stale author e-mail.
- [ ] Update `doc/examples.rst` Scenario 17 checkpoint table (~line 5620) and the
      `-0.87 %` / `+0.14 %` headline claims — those are pre-fix.
- [ ] `README.md`: "Final altitude error: **0.87 %**" was assumed pre-fix, but at
      the recommended dt = 1 ms the measured value *is* **0.87 %** (232.4 vs
      234.5 km). Check it against `doc/examples.rst` before changing anything.

Measured against sim 06 at dt = 1 ms:

| Quantity | Peak err | at t | Final err | NASA spread |
|---|---|---|---|---|
| Altitude MSL | 2.04 km | 200.0 | 2.04 km | 17.0 km |
| Down-range | 15.8 km | 200.0 | 15.8 km | 7.36 km |
| Pitch angle | 3.20° | 158.4 | 0.46° | 10.1° |
| Body pitch rate | 2.24°/s | 25.8 | 0.19°/s | 0.237°/s |
| Vehicle speed | 586 m/s | 193.0 | 11.5 m/s | 45.1 m/s |

Two things to know when reading these. Sims 04 and 05 are effectively the same
implementation (6.6 m apart over 200 s); sim 06 is the only independent second
reference, so the "spread" is really a two-way disagreement. And **down-range is
outside the reference envelope** (15.8 km vs 7.36 km) — an open discrepancy,
most likely the coast pitch rate (0.197 °/s here vs 0.163 °/s in the reference)
rather than the integrator. It is reported as open in the paper, not absorbed.

**The scenario is not step-converged, and it is not the integrator's fault.**
Re-running at dt = 10 ms moves the final altitude by 593 m and the peak altitude
error from 2.04 km to 1.45 km — a 29 % change for a 10x step change, i.e. first
order, not fifth. `RocketStageModel::advance` depletes fuel with explicit Euler
(`fuelUsed += mdot*dt`) and detects burnout only at step boundaries with no
interpolation to the crossing. Both are O(dt) and both act exactly at the events
that dominate the error table. Fixing it means event location on the
fuel-exhaustion crossing plus a mass quadrature of matching order. Until then,
quote Scenario 17 numbers *with* their step size — they are not comparable
across dt.


### 2. Regenerate the pre-fix figures — DONE

Every per-scenario figure is now post-fix, and every family has a checked-in
regeneration path:

```
python scripts/regenerate_atmos_figures.py            # scenarios 1,2,3,6,7,8,9,10
python scripts/plot_f16_s11_nasa02.py <sim.csv>       # and s12, s13p1..p4, s15, s16
python scripts/plot_atmos17_scenario17.py --sim <sim.csv> --ref <ref.csv> --output doc/_static/atmos17
python doc/_static/atmos17/generate_model_plots.py
```

- [x] Caveat in `doc/examples.rst` replaced with a statement of which script
      generates which family, and of the fact that the per-scenario figures
      compare against `sim_01` while the summary table quotes the closest run.

Two things that had been silently broken:

- `plot_f16_s*.py` wrote to `doc/figures/`, but the docs read `doc/_static/`.
  A previous regeneration therefore never reached the documentation. Fixed in
  all eight; the orphaned `doc/figures/` tree is deleted.
- Scenarios 1--10 had no regeneration script at all, which is why they were
  stuck. `scripts/regenerate_atmos_figures.py` is table-driven over the eight
  scenarios (they share run parameters, schema and figure set, unlike the F-16
  cases) and shells out to the existing `compare_sim_validation.py`. It forces
  UTF-8 on child processes: on a cp1254 console the children died with
  `UnicodeEncodeError` when their output was captured.

`doc/_static/atmos01` had only 10 of the 31 channels; it now has the full set
like the other scenarios.

### 3. Finish the per-scenario prose in `doc/examples.rst`

Figure **captions** for scenarios 1--10 are done: each altitude caption now
names the reference the figure actually uses (`sim_01`) and gives the
closest-run figure alongside, following the pattern scenario 6 already had.
Verified against freshly regenerated `error_summary.csv` files; the
closest-run and spread values reproduce the paper's summary table exactly for
scenarios 1, 3, 6, 7, 8, 9 and 10.

Corrected while doing it:

- scenario 1 altitude caption quoted the closest-run number (2.0e-5 m) against
  a figure drawn versus `sim_01` (4.7e-4 m);
- scenario 1 gravity caption said 4.7e-7 %, measured 2.9e-5 %;
- "reproduced to full double precision (relative error < 1e-10)" was off by
  three orders — measured 1.2e-7 versus `sim_01`, 2.2e-9 versus the closest run;
- scenarios 9 and 10 altitude figures had no caption at all.

Note: scenario 2 publishes only **one** reference run in SI form
(`Atmos_02_sim_01_si_units.csv`), so no closest-run or spread number can be
derived from the SI files alone. Its caption says so rather than quoting a
spread of zero.

Still open:

- [ ] Per-checkpoint **tables** and summary sentences for scenarios 8, 9, 10,
      11, 12, 13.1--13.4, 15, 16 have not been re-verified against fresh runs.
      Scenario 1's table was checked and is correct; scenario 17's was checked
      and is correct.

### 4. Confirm the ecos failures are pre-existing

`feed_through_values_ecos_ecos`, `feed_through_zero_values_ecos_ecos`,
`reset_to_defaults_ecos_ecos` segfault. They exercise an FMI feed-through model
with no Aetherion dynamics in it, so they should be unrelated — but verify on
`main` before claiming that in the paper's CI table.

### 5. Papers — DONE

- [x] `papers/eucass/main.tex` — TÜBİTAK placeholder removed (there is no
      TÜBİTAK grant); author e-mail corrected to `onur.tuncer@itu.edu.tr`.
- [x] `papers/eucass-library/main.tex` — same, plus the Scenario 17 figure,
      table row and prose.
- [ ] Neither paper has been compiled. No LaTeX toolchain was available in this
      session — run `pdflatex`/`bibtex` and fix whatever falls out. Paper 2 uses
      `listings`, `multirow`, `siunitx` (`\SIrange`, `\num`); paper 1 now uses
      `\Ad`, which is defined in its preamble.
- [ ] Decide whether the trivialisation result deserves to lead paper 1. It is
      currently contribution (4) of 5; it may be the most useful thing in the
      paper to a reader implementing RKMK, in which case the title and abstract
      should say so more directly.

### 6. Housekeeping

- [ ] `doc/conf.py` gained an `\Ad` MathJax macro — keep.
- [ ] Consider whether `SE3::dexp_inv` should grow a companion
      `dexp_inv_left(eta)` (= `dexp_inv(-eta)`) so call sites cannot get the sign
      wrong again. The current fix negates at each of the three call sites, which
      works but is repeated.
- [ ] Ninja does not detect header changes in this environment — MSVC's
      `/showIncludes` output is localised (Turkish, "Not: eklenen dosya"), so the
      dependency scan yields nothing. **After editing a header, delete the
      target's `CMakeFiles/<target>.dir` or use `--clean-first`**, otherwise you
      will test a stale binary. This cost an hour today. Consider setting
      `VSLANG=1033` in the environment or the CMake presets.
