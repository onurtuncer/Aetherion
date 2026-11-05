# Mutation testing: replay pilot

This opt-in Mull integration asks whether the existing Catch2 estimation tests
notice deliberately changed production logic. It complements coverage and the
normal regression/sanitizer jobs; it does not change release builds.

## Scope and toolchain

The initial scope is `Aetherion/Estimation/Replay/{Csv,Logs,TimeBase}.h`.
Arithmetic (including compound assignment), comparisons and boundary operators
are mutated. Each mutant runs the `test_estimation` cases that include those
headers, selected by the Catch2 filter `[replay]~[e2e]` (`TEST_FILTER` in
`run.py`); test code, Eigen and other dependencies are excluded. The EKF,
process- and measurement-model tests cannot see these mutants, and the 20-seed
`[e2e]` replay, half of the suite's runtime, exercises the same readers that the
other `[replay]` cases test directly. These three headers are
implemented inline, so instrumentation belongs on `test_estimation`, not on the
Aetherion library. Expanding to compiled implementation files requires instrumenting
their owning targets too.

Supported pilot environment: Ubuntu 24.04, Clang **19**, Mull **0.28.0 / LLVM 19**.
The workflow downloads a version-specific Mull release package; no `latest` URL
or remote installation script is used. See [Mull installation](https://mull.readthedocs.io/en/0.28.0/Installation.html).
Install Clang 19, CMake >= 3.26, Ninja, libtbb-dev and the matching Mull package.
The regular vcpkg dependency setup is still used.

## Run locally

From the repository root (with the ecos submodule initialized):

```sh
git submodule update --init --recursive
cmake -S . -B build/mutation -G Ninja \
  -DCMAKE_BUILD_TYPE=Debug -DCMAKE_CXX_COMPILER=clang++-19 \
  -DAETHERION_ENABLE_MUTATION_TESTING=ON \
  -DAETHERION_BUILD_TESTS=ON \
  -DAETHERION_BUILD_FMUS=OFF -DAETHERION_BUILD_EXAMPLES=OFF \
  -DAETHERION_BUILD_SIMULATION=OFF
cmake --build build/mutation --target mutation --parallel 2
```

For non-default installations, set `AETHERION_MULL_PLUGIN` and
`AETHERION_MULL_RUNNER` at configure time. Use a fresh build directory, without
sanitizer, gcov, LTO or other instrumentation flags. The pilot uses `-O0 -g` and
records compiler commands. `MULL_CONFIG` is explicitly passed at both compilation
and execution; changing the configuration rebuilds the instrumented sources.
No coverage filtering is enabled: uncovered mutants remain visible in the report.

## CI and reports

`.github/workflows/mutation.yml` runs on relevant pull requests, on pushes to
main that touch the same paths, weekly on main, and on manual dispatch. It is
separate from the regular builds. On main, a successful run publishes the score
as `mutation.json` (shields.io endpoint format, written by `run.py` as
`badge.json`) to the orphan `badges` branch, which the top-level README's badge
reads; pull requests never publish. Only the separate `publish-badge` job has
write access. Survivors are
advisory during rollout; baseline failures, runner failures, missing reports,
empty reports, and reports with no executed mutants fail the job. Do not mark
this job as a required score gate until the baseline has been reviewed.

`build/mutation/mutation-report/` contains baseline and Mull logs, Elements
JSON/HTML, SQLite details and a Markdown summary. CI uploads these even after a
failure. The HTML viewer may require internet access to load its viewer script.
The score is for the three pilot headers only. Mull's Elements reporter groups
crashes and timeouts under `Killed`; inspect SQLite execution details before
crediting those as useful detections. The external baseline and Mull's own
unmutated warm-up/baseline each have a 5-minute timeout. Mull 0.28's `--timeout`
controls these unmutated runs too: a 30-second value aborts a healthy full
estimation suite before mutants start. Individual covered mutants use Mull
0.28's baseline-derived limit (`max(30 ms, 10 * baseline runtime)`), not a fixed
30-second cap — so the filtered test set *is* the per-mutant timeout. With the
whole suite (~70 s at `-O0`) a looping mutant ran for ~12 minutes while Mull
buffered its output, and the hosted runner lost contact (November 2026); with
the filter it is a few seconds. `--no-output` keeps Mull from buffering mutant
output at all. The entire Mull process has a 90-minute timeout; the CI job has
a 120-minute cap including dependency installation and compilation. A timeout
or nonzero subprocess exit remains a failure. On failure, the runner prints the
last 40 log lines to the CI console and retains the complete log in the artifact.

Mutants run with **one worker** because existing tests use fixed temporary file
names. Run only one campaign at a time on a shared machine. Do not change this
until those fixtures are isolated per process.

Review survivors by source location: add a regression test for a meaningful
missing assertion; document genuinely equivalent mutants before excluding them.
Never loosen numerical tolerances simply to improve the score. After a stable
baseline, consider a score floor and expand to truth interpolation, EKF covariance
checks and Lie-group operations in separate bounded campaigns.

Runner safeguards can be checked without a C++ toolchain:

```sh
python -m unittest discover -s tools/mutation -p 'test_*.py' -v
```
