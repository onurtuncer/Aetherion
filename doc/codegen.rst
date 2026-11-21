.. ------------------------------------------------------------------------------
.. Project: Aetherion
.. Copyright (c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
..
.. SPDX-License-Identifier: MIT
.. License-Filename: LICENSE
.. ------------------------------------------------------------------------------

.. _codegen:

Code Generation and Golden Vectors
==================================

.. contents::
   :local:
   :depth: 2

The double-precision reference filter (``InvariantEKF`` in
``Aetherion/Estimation/EKF.h``) is the specification of the estimator. A
flight computer cannot run it: it is templated C++ on dynamic Eigen matrices
and CppAD. Step 8 of the sensor-fusion plan provides the two artefacts that
carry the specification to embedded code:

* a **generator** (``generate_ekf_c``) that writes the filter's model
  functions for one vehicle configuration as plain C, together with a header
  that carries the error-state dimension, the block offsets and the
  configuration hash;
* **golden vectors**: a recorded run of the reference filter, inputs and
  outputs, with the tolerances an implementation is held to and the same
  hash.

The theory behind the functions (the group :math:`SE_2(3)\times\mathbb{R}^7`,
the LCI frame, the error conventions, the process and measurement models) is
in :doc:`sensor_fusion`. This page is about the tools.

.. important::

   **Generated C is never published from this repository.** CppADCodeGen is
   dual-licensed EPL-1.0 or GPL-3.0 and makes no statement about the code it
   generates. Generated C and headers therefore exist only in build trees:
   they are not committed here, not installed and not part of a release, and
   the generator executable is not shipped either. They are delivered to
   Hemerion, which checks them in under its own licence and banner. What this
   repository keeps, under MIT, is the generator source and the golden vectors
   (data). The CTest check ``estimation_generated_code_not_published`` enforces
   this (see :ref:`codegen-not-published`).


What is generated
-----------------

For a configuration :math:`\mathcal{C}` with error dimension :math:`n` and
noise dimension :math:`m`, and one error convention (left- or
right-invariant), ``generate_ekf_c`` writes six functions, each with the same
signature,

.. code-block:: c

   void <stem>_<name>(const double* x, double* y);

reading a flat input vector ``x`` and writing a flat output vector ``y``.
Each function is the reference filter's own model code taped with
``CppAD::AD<CppAD::cg::CG<double>>``, not a second derivation:

.. list-table::
   :header-rows: 1
   :widths: 16 38 46

   * - Function
     - Inputs (in order)
     - Outputs (in order)
   * - ``propagate``
     - state, gyro (3), accel (3), dt, frame
     - state after one IMU step (:math:`f`; ``StrapdownProcessModel::propagate``)
   * - ``transition``
     - state, gyro, accel, dt, frame, ``qc_diag`` (:math:`m`)
     - :math:`\Phi` (:math:`n\times n`), :math:`Q_d` (:math:`n\times n`) of
       ``step<C>``: the Jacobian side of :math:`f`
   * - ``gnss_position``
     - state, :math:`y` (3, LCI), measurement covariance (3 x 3, LCI)
     - innovation :math:`z` (3), :math:`H` (:math:`3\times n`), :math:`R` (3 x 3)
   * - ``gnss_velocity``
     - as ``gnss_position``
     - as ``gnss_position``
   * - ``barometer``
     - state, :math:`y` (pressure altitude, m), :math:`\sigma` (m), frame
     - :math:`z` (1), :math:`H` (:math:`1\times n`), :math:`R` (1)
   * - ``retract``
     - state, :math:`\delta` (:math:`n`)
     - state corrected by :math:`\delta` (``retract<C>``)

A measurement function is generated only if the configuration carries the
measurement. :math:`H` and :math:`R` are in the convention's innovation
coordinates, exactly as ``MeasurementModels<S>`` returns them, so the
covariance algebra around them is the ordinary EKF update; the generated code
does not contain it, because the numerical form of the covariance (Joseph,
UD or square root) is a step-7b decision that belongs to the embedded filter.

Packing, the same in every function:

* **state** (22): :math:`R` (body to LCI, 9, row-major), :math:`v`, :math:`p`
  (LCI), gyro bias, accelerometer bias, barometer bias. A state always holds
  every library block; which of them are estimated is the configuration's
  business.
* **frame** (12): ECI position of the LCI origin, then :math:`R_{WL}`
  (LCI to ECI, row-major). The Earth rotation angle is not an input: GNSS
  measurements enter already mapped to LCI, as at the ``InvariantEKF``
  interface.
* **matrices** are row-major everywhere.

The generated header names every offset, so the consumer never writes an
index by hand:

.. list-table::
   :header-rows: 1
   :widths: 45 55

   * - Macro
     - Meaning
   * - ``<STEM>_CONFIG_HASH``, ``_CONFIG_HASH_STRING``, ``_CONFIG_NAME``
     - FNV-1a 64 of the configuration descriptor (``Configuration::hash``)
   * - ``<STEM>_ERROR_CONVENTION``, ``_ERROR_CONVENTION_RIGHT_INVARIANT``
     - the convention the functions linearise in
   * - ``<STEM>_ERROR_DIM``, ``_NOISE_DIM``
     - :math:`n`, :math:`m`
   * - ``<STEM>_BLOCK_<NAME>_OFFSET``, ``_DIM``
     - each carried block's position in the error vector, in canonical order
   * - ``<STEM>_ERROR_ATTITUDE``, ``_VELOCITY``, ``_POSITION``
     - :math:`\phi`, :math:`\nu`, :math:`\rho` inside the error vector
   * - ``<STEM>_NOISE_..._OFFSET``
     - layout of the noise vector (``qc_diag``)
   * - ``<STEM>_STATE_...``, ``_FRAME_...``
     - packing of the state and the frame
   * - ``<STEM>_<FUNCTION>_N_IN``, ``_N_OUT``, ``_N_TMP``
     - sizes of ``x`` and ``y``; doubles of automatic storage used
   * - ``<STEM>_<FUNCTION>_IN_<SEGMENT>``, ``_OUT_<SEGMENT>``
     - offset of each named input and output segment

The header also repeats the configuration descriptor (the hash input) as a
comment, so a reader can see what the hash names.

Every generated file begins with the banner the caller supplies, followed by
the provenance lines: a marker line, the generator's commit, CppADCodeGen's
version, and the configuration, convention and hash.


Tutorial
--------

1. Build with CppADCodeGen
~~~~~~~~~~~~~~~~~~~~~~~~~~

CppADCodeGen is an optional, host-side dependency (vcpkg manifest feature
``codegen``), selected by ``AETHERION_WITH_CPPADCG``. The ``release`` and
``gcc-release`` presets turn it on:

.. code-block:: console

   cmake --preset=release
   cmake --build --preset=release --target generate_ekf_c generate_golden_vectors

Without vcpkg, put CppADCodeGen 2.5.0 on ``CMAKE_PREFIX_PATH`` and configure
with ``-DAETHERION_WITH_CPPADCG=ON``. CppADCodeGen is linked by the generator
and its tests only, never by the ``Aetherion`` library, the examples or the
FMUs. On MSVC the generator includes a source-generation subset of
CppADCodeGen (``tests/Estimation/codegen/CppADCodeGenCore.h``) because
``<cppad/cg.hpp>`` 2.5.0 does not compile there.

``generate_golden_vectors`` needs no CppADCodeGen and is built in every test
build.

2. Generate the C for a configuration
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Write the banner your project wants at the top of the files (plain text, one
line per line; it becomes a C comment), then run the generator from the build
tree:

.. code-block:: console

   generate_ekf_c --out <dir> --banner <banner.txt> \
                  --generator-commit <aetherion sha> --cppadcg-version 2.5.0 \
                  [--config nav-baro-gnss16] [--convention ri|li] [--stem aeth_ekf]

* ``--banner`` is required. The generator never writes a licence of its own.
* ``--generator-commit`` and ``--cppadcg-version`` are recorded in the files;
  pass the Aetherion commit the tool was built from and the CppADCodeGen
  version of the build.
* ``--config`` names the configuration; ``nav-baro-gnss16``
  (``kNavBaroGnss16``) is the one that exists.
* ``--convention`` defaults to ``kErrorConvention`` in ``Configuration.h``,
  the one the pinned hash names (right-invariant, provisional).
* ``--stem`` prefixes the file names, the functions and the macros.

It writes ``<dir>/<stem>.c`` and ``<dir>/<stem>.h`` and prints, per function,
the tape size, the number of temporaries and the bytes of C.

3. Compile and call the functions
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The C file needs only ``<math.h>`` and the header; it is C99 and uses no heap.
A prediction and a GNSS position update look like this; ``P`` and the
covariance algebra are the caller's:

.. code-block:: c

   #include <string.h>

   #include "aeth_ekf.h"

   double x[AETH_EKF_STATE_DIM];              /* nominal state */
   double frame[AETH_EKF_FRAME_DIM];          /* LCI origin and rotation */
   double qc_diag[AETH_EKF_NOISE_DIM];        /* continuous noise PSD diagonal */

   void predict(const double gyro[3], const double accel[3], double dt)
   {
       double in[AETH_EKF_TRANSITION_N_IN];
       double x1[AETH_EKF_PROPAGATE_N_OUT];
       double out[AETH_EKF_TRANSITION_N_OUT];
       /* The transition inputs begin with the propagate inputs. */
       memcpy(in + AETH_EKF_TRANSITION_IN_STATE, x, sizeof x);
       memcpy(in + AETH_EKF_TRANSITION_IN_GYRO, gyro, 3 * sizeof(double));
       memcpy(in + AETH_EKF_TRANSITION_IN_ACCEL, accel, 3 * sizeof(double));
       in[AETH_EKF_TRANSITION_IN_DT] = dt;
       memcpy(in + AETH_EKF_TRANSITION_IN_FRAME, frame, sizeof frame);
       memcpy(in + AETH_EKF_TRANSITION_IN_QC_DIAG, qc_diag, sizeof qc_diag);

       aeth_ekf_propagate(in, x1);
       aeth_ekf_transition(in, out);
       /* Phi = out + AETH_EKF_TRANSITION_OUT_PHI, Qd = out + ..._OUT_QD:
          P <- Phi P Phi^T + Qd, in the embedded filter's covariance form. */
       memcpy(x, x1, sizeof x);
   }

   void update_gnss_position(const double y_lci[3], const double cov_lci[9])
   {
       double in[AETH_EKF_GNSS_POSITION_N_IN], out[AETH_EKF_GNSS_POSITION_N_OUT];
       memcpy(in + AETH_EKF_GNSS_POSITION_IN_STATE, x, sizeof x);
       memcpy(in + AETH_EKF_GNSS_POSITION_IN_Y, y_lci, 3 * sizeof(double));
       memcpy(in + AETH_EKF_GNSS_POSITION_IN_MEAS_COV, cov_lci, 9 * sizeof(double));
       aeth_ekf_gnss_position(in, out);
       /* z, H, R at out + ..._OUT_Z, _OUT_H, _OUT_R. Compute K and delta = K z,
          then x <- aeth_ekf_retract([x, delta]) and update P. */
   }

The generated functions evaluate both sides of every small-angle switch and
select one with an ``if``; both sides are finite everywhere they are
evaluated (see the ``CondExp`` lesson in :doc:`sensor_fusion`), so no input
the reference filter accepts produces a NaN in a dead branch.

Before using the code, compare its hash with the one in the golden vectors
and refuse to run on a mismatch: ``AETH_EKF_CONFIG_HASH`` must equal the
``hash`` line of the file.

4. Produce the golden vectors
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: console

   generate_golden_vectors --out data/Estimation/GoldenVectors \
                           [--config nav-baro-gnss16] [--convention ri|li] [--duration 1.0]

writes ``<config>_<convention>.txt``, for example
``data/Estimation/GoldenVectors/nav-baro-gnss16_right-invariant.txt``. The
sequence drives the reference filter through every call it has:

* the banking, pitching synthetic truth of the estimator tests
  (``tests/Estimation/SyntheticTruth.h``) from :math:`t = 20` s, in an LCI
  frame at 41.1 N, 29.0 E;
* a large initial error (2, -1.5 and 4 degrees of attitude, about 1 m/s and
  8 m), so the first updates make large corrections;
* an IMU at 100 Hz with constant bias and white noise at the replay's default
  densities;
* GNSS position and velocity at 10 Hz with a full 3 x 3 covariance, whose
  validity times fall inside an IMU interval, so the predict is split there
  as the replay harness does;
* a barometer at 25 Hz with a 5 m bias.

The inputs are drawn by the tool (a fixed seed and a platform-independent
Gaussian); the outputs are whatever ``InvariantEKF<double, C>`` computes from
them. The files are committed. Regenerate them only when the reference filter
changes deliberately, and say so: Hemerion's replay test reads them.

5. The file format
~~~~~~~~~~~~~~~~~~

Text, one item per line, comma-separated, ``#`` starts a comment. Numbers have
17 significant digits, which round-trips a double. The header gives the
format version, the configuration, the convention, the ``hash``, the
dimensions, one ``tolerance`` line per checked quantity, the ``frame``, the
``qc_diag`` and the ``initial`` state and covariance. Then, in call order,
each record is an input line (``predict``, ``gnss_position``,
``gnss_velocity`` or ``barometer``), for an update an ``innovation`` line
(NIS, then :math:`z`), and a ``state`` line: the state and the upper triangle
of :math:`P`, row by row, after the call. The reader, the writer and the
comparison are in ``tests/Estimation/golden/GoldenVectors.h``.

6. Checking an implementation
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Replay the file's inputs through the implementation, starting from its
``initial`` line, and compare after every call. For each quantity :math:`q`
with tolerance ``tolerance,<set>,q,abs,rel``, the largest absolute deviation
:math:`d` over the quantity's entries must satisfy

.. math::

   d \le \mathrm{abs} + \mathrm{rel}\cdot r,

with :math:`r` the largest reference entry of that state block, of :math:`z`
or of the NIS; for the covariance the test is entry by entry with
:math:`r = \sqrt{P_{ii}P_{jj}}` of the reference. A deviation that is not a
number fails. The set ``float64`` applies to a double-precision
implementation. There is no ``float32`` set yet: it waits for the step-7b
choice of covariance form.

.. list-table:: Tolerances of the ``float64`` set
   :header-rows: 1
   :widths: 30 20 20

   * - Quantity
     - abs
     - rel
   * - attitude (:math:`R` entries)
     - 1e-10
     - 0
   * - velocity (m/s)
     - 1e-9
     - 1e-10
   * - position (m)
     - 1e-8
     - 1e-10
   * - gyro bias (rad/s)
     - 1e-11
     - 1e-8
   * - accel bias (m/s²)
     - 1e-10
     - 1e-8
   * - baro bias (m)
     - 1e-9
     - 1e-8
   * - covariance
     - 0
     - 1e-8
   * - innovation
     - 1e-8
     - 1e-8
   * - NIS
     - 1e-8
     - 1e-8

The basis for these numbers is in the case study below: an independent
double-precision filter built on the generated C stays at least ten times
inside every one of them, and a 1e-6 relative error in every time step fails
every one of them, by factors of 9.6 to 4232.


.. _codegen-not-published:

Keeping generated code out of this repository
---------------------------------------------

In the build, ``generate_ekf_c`` runs into
``<build>/tests/Estimation/generated_ekf/`` and the C is compiled into the
test binary ``test_estimation_codegen``. Nothing installs it. The CTest test
``estimation_generated_code_not_published``
(``cmake/CheckGeneratedEstimatorCodeNotPublished.cmake``) fails if

* any file in the git index carries the marker line every generated file
  begins with, or
* any install script or install manifest of the build tree names the
  generated directories, files or generator executables.


Case study: ``kNavBaroGnss16``
------------------------------

The first configuration, end to end: 16 error states (navigation core 9, gyro
bias 3, accelerometer bias 3, barometer bias 1), updated by GNSS position,
GNSS velocity and barometer, in both error conventions.

**Run.** Branch ``feat/ekf-codegen-golden-vectors`` from ``main`` at
``7db2b06``; Windows 11, MSVC 19.50 (x64), preset ``release`` (Ninja,
``/O2``), CppADCodeGen 2.5.0 from vcpkg. Generated in the build by the rule in
``tests/Estimation/CMakeLists.txt``, which is the same as

.. code-block:: console

   generate_ekf_c --out generated_ekf --stem aeth_ekf_ri --config nav-baro-gnss16 \
                  --convention ri --banner tests/Estimation/codegen/test_banner.txt \
                  --generator-commit build-tree-test --cppadcg-version 2.5.0

and the same with ``li``; the golden vectors by
``generate_golden_vectors --out data/Estimation/GoldenVectors --convention ri``
and ``li``, default duration 1 s.

**Configuration.** Hash ``0x1e59a990edd19e83`` (right-invariant, the pinned
value of ``test_Configuration.cpp``) and ``0x9055067ccc7acb60``
(left-invariant). Header: ``ERROR_DIM 16``, ``NOISE_DIM 13``, block offsets
nav core 0, gyro bias 9, accel bias 12, baro bias 15.

**Generated sizes.** Generation takes 0.06–0.21 s per convention.

.. list-table::
   :header-rows: 1
   :widths: 22 10 10 18 18 22

   * - Function
     - in
     - out
     - tape variables RI / LI
     - temporaries RI / LI
     - bytes of C RI / LI
   * - ``propagate``
     - 41
     - 22
     - 437 / 437
     - 18 / 18
     - 5010 / 5010
   * - ``transition``
     - 54
     - 512
     - 2538 / 2022
     - 95 / 70
     - 31045 / 23040
   * - ``gnss_position``
     - 34
     - 60
     - 89 / 191
     - 0 / 3
     - 1059 / 1972
   * - ``gnss_velocity``
     - 34
     - 60
     - 89 / 191
     - 0 / 3
     - 1056 / 1971
   * - ``barometer``
     - 36
     - 18
     - 160 / 163
     - 6 / 6
     - 1623 / 1632
   * - ``retract``
     - 38
     - 22
     - 341 / 341
     - 21 / 14
     - 3825 / 3858

Files: ``aeth_ekf_ri.c`` 44 246 bytes, ``aeth_ekf_ri.h`` 5 681;
``aeth_ekf_li.c`` 38 110, ``aeth_ekf_li.h`` 5 678. Running the generator
twice gives byte-identical files. The RI ``transition`` is the largest
because the right-invariant bias coupling :math:`-\mathrm{Ad}_{\hat X}` and
the :math:`[p]_\times` terms are state-dependent; the LI GNSS functions are
larger because :math:`z = \hat R^\top(y - \hat p)` and
:math:`R = \hat R^\top \Sigma \hat R` rotate.

As a portability check, both C files compile with
``arm-none-eabi-gcc`` 14.2.1 (``-std=c99 -Wall -Wextra -Wpedantic
-Wdouble-promotion -Wconversion -O2 -mcpu=cortex-m7 -mfpu=fpv5-d16
-mfloat-abi=hard``) without a warning, to 20 384 (RI) and 17 312 (LI) bytes
of text.

**Generated values against the double reference**
(``test_EkfCodeGen.cpp``). Every function at two points that between them
take both sides of every small-angle switch (point A: :math:`|\omega\,dt|
\approx 10^{-3}`, :math:`\delta = 0`; point B: :math:`|\omega\,dt| \approx
2\times10^{-6}`, a 0.25 rad :math:`\delta`), each in the frame the code was
taped in and in a second frame (33.9 S, 151.2 E), so no frame value is frozen
into the code. Worst relative difference per output segment:

.. list-table::
   :header-rows: 1
   :widths: 30 35 35

   * - Function
     - RI
     - LI
   * - ``propagate``
     - 4.1e-21
     - 4.1e-21
   * - ``transition``
     - 1.1e-16
     - 5.6e-24
   * - ``gnss_position``
     - 0
     - 4.9e-17
   * - ``gnss_velocity``
     - 0
     - 4.9e-17
   * - ``barometer``
     - 2.4e-16
     - 1.6e-16
   * - ``retract``
     - 0
     - 0

The bound is 1e-12. The innovation is measured relative to the size of the
inputs, because it is a difference of terms that size: the barometer's
:math:`h(x)` passes through the ECI position (about 6.4e6 m), and the
generated and reference barometer innovations differ by 9.3e-10 m, one unit
in the last place of 6.4e6 m, which is 4.5e-11 of a 20.8 m innovation.
Scaling the largest entry of a generated :math:`\Phi` by
:math:`1 + 10^{-10}` fails the check.

**Generated Jacobians against CppAD.** At point A, :math:`H` from the
generated measurement functions against ``CppAD::AD<double>`` derivatives
:math:`\partial z/\partial\delta` of the nonlinear maps (relative norm of the
difference): RI GNSS position and velocity 0, barometer 3.2e-17; LI GNSS
1.3e-16, barometer 8.8e-17. :math:`\Phi` from ``transition`` against the
AD derivative of the nonlinear error map over one step: 2.8e-7 (RI) and
9.1e-7 (LI). That difference is the discretisation of the trapezoidal
:math:`\Phi`, not round-off: it is the same order the reference model has
against the same derivative (``test_ProcessModel.cpp`` establishes it as a
convergence order, :math:`O(dt^3)`).

**Golden vectors.** 155 records per convention over 1 s: 110 predicts (20 of
them partial, around the 10 GNSS epochs), 10 GNSS position, 10 GNSS velocity
and 25 barometer updates; 557 kB (RI) and 578 kB (LI). Rerunning the
generator reproduces both files byte for byte, and the reference filter
reproduces them exactly (``test_GoldenVectors.cpp``).

An independent filter made only of the generated C, with its own covariance
algebra (an explicit inverse of the innovation covariance instead of the
reference's LDLT solve), replays them within tolerance. Largest deviation as
a fraction of the allowed one, over all 155 records:

.. list-table::
   :header-rows: 1
   :widths: 30 35 35

   * - Quantity
     - RI
     - LI
   * - attitude
     - 0.051
     - 0.0014
   * - velocity
     - 6.3e-4
     - 2.7e-4
   * - position
     - 1.3e-3
     - 8.7e-5
   * - gyro bias
     - 4.4e-4
     - 3.4e-5
   * - accel bias
     - 1.9e-3
     - 1.1e-3
   * - baro bias
     - 8.1e-3
     - 6.2e-3
   * - covariance
     - 1.4e-3
     - 6.7e-6
   * - innovation
     - 0.092
     - 0.10
   * - NIS
     - 0.091
     - 0.074

The worst is 0.10: at least a factor of ten of margin for a correct
double-precision implementation. The comparison is shown to fail when it
should:

* every predict's :math:`dt` scaled by :math:`1 + 10^{-6}`: every quantity
  fails, from 9.6 (RI barometer bias) to 4232 (LI attitude) times its
  tolerance; RI attitude 4156, NIS 3316, innovation 1579, position 243,
  covariance 169;
* the left-invariant filter on the right-invariant file: every quantity
  between 2.7e6 and 6.4e8 times its tolerance;
* one gyro reading off by 1e-6 rad/s in the first predict (reference
  filter): attitude 88, NIS 8.0, accelerometer bias 5.1 times their
  tolerances;
* a NaN in one covariance entry.

**Contract.** The test refuses a golden file whose ``hash`` differs from the
``CONFIG_HASH`` of the generated code it is run against, which is the check
Hemerion's replay test makes (its step 9).


Source files
------------

.. list-table::
   :header-rows: 1
   :widths: 50 50

   * - File
     - Role
   * - ``tests/Estimation/codegen/EkfMaps.h``
     - the generated functions as maps on a scalar ``S``: layouts and the
       double reference
   * - ``tests/Estimation/codegen/generate_ekf_c.cpp``
     - the generator tool
   * - ``tests/Estimation/codegen/test_EkfCodeGen.cpp``
     - generated C against the reference, CppAD and the golden vectors
   * - ``tests/Estimation/golden/GoldenVectors.h``
     - golden-vector format, reference run, comparison
   * - ``tests/Estimation/golden/generate_golden_vectors.cpp``
     - writes the golden vectors
   * - ``tests/Estimation/test_GoldenVectors.cpp``
     - the committed files against the reference filter
   * - ``data/Estimation/GoldenVectors/``
     - the golden vectors
   * - ``cmake/CheckGeneratedEstimatorCodeNotPublished.cmake``
     - the not-published check
   * - ``tests/Estimation/codegen/CodeGenModels.h``,
       ``generate_estimation_cg.cpp``, ``test_CodeGen.cpp``
     - the earlier CppADCodeGen compatibility check (process error map and
       dense Jacobians), kept as is
