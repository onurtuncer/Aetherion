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

### 1. Scenario 17 is INCOMPLETE — redo it first

The two-stage rocket run **did not finish**: the background task timed out and
the CSV only reaches **t = 31.4 s of 200 s**. Do not use any Scenario 17 number
currently in hand.

```
out\build\windows-release\src\Examples\TwoStageRocket\TwoStageRocket.exe \
    --outputFileName atmos17.csv --startTime 0 --endTime 200 \
    --timeStep 0.001 --writeInterval 100
```

200 000 Radau steps — allow well over 15 minutes, run it detached and let it
finish. Then:

- [ ] Recompute vs `Atmos_17_sim_04/05/06`.
- [ ] Fill the Scenario 17 row in **both** papers' verification tables
      (`\TODO{X}` markers) and the Scenario 17 error table in
      `papers/eucass/main.tex` (~line 907).
- [ ] Update `doc/examples.rst` Scenario 17 checkpoint table (~line 5620) and the
      `-0.87 %` / `+0.14 %` headline claims — those are pre-fix.
- [ ] Also fix `README.md`: "Final altitude error: **0.87 %**" is pre-fix.
- [ ] Reconcile the two comparison scripts — for the partial atmos17 data they
      disagreed (9.59 m vs 16.89 m max |dh| over the same window). Find out why
      before trusting either.

Scratch scripts (regenerate if the temp dir is gone):
`<scratchpad>/run_validation.py`, `compare_all.py`, `summary.py`.

### 2. Regenerate the pre-fix figures

Everything under `doc/_static/atmos*/` and `doc/_static/f16_*/` was generated
before the fix. The summary table in `_reference_run_spread` is current, and a
note there says so, but the figures should be regenerated for consistency:

```
python scripts/plot_atmos17_scenario17.py <sim.csv>
python scripts/plot_f16_s11_nasa02.py <sim.csv>      # and s12, s13p1..p4, s15, s16
python doc/_static/atmos17/generate_model_plots.py
```

- [ ] Then drop the "figures are being regenerated" caveat from the note in
      `doc/examples.rst`.

### 3. Finish the per-scenario prose in `doc/examples.rst`

Scenarios 8, 9, 10, 11, 12, 13.1–13.4, 15, 16 still have pre-fix numbers in
their per-checkpoint tables and summary sentences. The values are in the
`_reference_run_spread` table; the per-scenario text needs to be brought into
line with it.

### 4. Confirm the ecos failures are pre-existing

`feed_through_values_ecos_ecos`, `feed_through_zero_values_ecos_ecos`,
`reset_to_defaults_ecos_ecos` segfault. They exercise an FMI feed-through model
with no Aetherion dynamics in it, so they should be unrelated — but verify on
`main` before claiming that in the paper's CI table.

### 5. Papers

- [ ] `papers/eucass/main.tex` — remove `\TODO` for the TÜBİTAK grant number or
      fill it in.
- [ ] `papers/eucass-library/main.tex` — same; plus the Scenario 17 figure.
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
