.. ------------------------------------------------------------------------------
.. Project: Aetherion
.. Copyright (c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
..
.. SPDX-License-Identifier: MIT
.. License-Filename: LICENSE
.. ------------------------------------------------------------------------------

.. _daveml:

DAVE-ML Models
==============

Overview
--------

**DAVE-ML** (Dynamic Aerospace Vehicle Exchange Markup Language) is an
XML-based standard for encoding aerodynamic databases, flight-dynamics
models, and control-system definitions in a tool-neutral, human-readable
format. It is governed by AIAA Standard S-119 and uses
**MathML 2.0 Content** markup to express both lookup-table interpolation
and arbitrary algebraic calculations, while embedding self-contained
check-cases that allow any compliant reader to validate its implementation
against the original data source.

A DAVE-ML file (``.dml``) contains four principal element types:

- ``<variableDef>`` — declares every scalar quantity (constant, input,
  output, or intermediate calculated variable) with its identifier,
  units, description, and optional ``initialValue`` or MathML body.
- ``<breakpointDef>`` — defines a sorted vector of independent-variable
  breakpoints used to index gridded tables.
- ``<griddedTableDef>`` — stores a row-major, multi-dimensional lookup
  table and references the ``<breakpointDef>`` entries that span each
  axis.
- ``<functionDef>`` — binds a ``<griddedTableDef>`` to a set of input
  ``<variableDef>`` identifiers, making the table callable by name in
  MathML expressions.

The Aetherion project uses DAVE-ML exclusively to encode the **F-16
Fighting Falcon** aerodynamic, propulsion, inertia, and control-law data
that originated in Stevens & Lewis [SL03]_ and were adapted by Morelli
for NASA TM-2003-212145 [Mo03]_.

AIAA S-119 conformance level supported: **DAVE-ML 2.0**.


C++ Architecture
----------------

The DAVE-ML subsystem lives under ``Aetherion/Serialization/DAVEML/`` and
provides four public interfaces.

.. _daveml_reader:

DAVEMLReader
~~~~~~~~~~~~

A **lightweight, zero-dependency XML reader** for ``.dml`` files that
contain only constant ``<variableDef>`` elements and simple MathML
arithmetic (no gridded tables, no ``<piecewise>`` branching, no
comparisons).

*Supported MathML operators:*
``<times>``, ``<plus>``, ``<minus>``, ``<divide>``, ``<power>``,
``<cn>`` (numeric literal), ``<ci>`` (variable reference).

*Typical use:* loading inertial properties from ``F16_inertia.dml``.

.. code-block:: cpp

   #include "Aetherion/Serialization/DAVEML/DAVEMLReader.h"

   DAVEMLReader r("data/F16_S119_source/F16_inertia.dml");

   double Ixx = r.getValueSI("XIXX");      // kg·m²
   double mass = r.getValueSI("XMASS");    // kg

   // Re-evaluate calculated outputs at a new input value.
   r.setInput("CG_PCT_MAC", 30.0);
   double dx = r.getValue("DXCG");         // ft (native units)

Key methods:

.. list-table::
   :header-rows: 1
   :widths: 40 60

   * - Method
     - Description
   * - ``getValue(varID)``
     - Return the variable's value in its native (file) units.
   * - ``getValueSI(varID)``
     - Return the value after converting to SI
       (slug→kg, slug·ft²→kg·m², ft→m, pct→–).
   * - ``setInput(varID, value)``
     - Override an input variable and re-evaluate all downstream
       calculated variables that depend on it.
   * - ``hasVar(varID)``
     - Return ``true`` if the identifier is present in the file.

Unit conversion factors applied by ``getValueSI``:

.. list-table::
   :header-rows: 1
   :widths: 20 20 60

   * - DAVE-ML units
     - SI units
     - Factor
   * - ``slug``
     - kg
     - 14.593 902 937
   * - ``slugft2``
     - kg·m²
     - 1.355 817 948
   * - ``ft``
     - m
     - 0.304 8
   * - ``pct``
     - (dimensionless)
     - 0.01


.. _daveml_aero:

DAVEMLAeroModel
~~~~~~~~~~~~~~~

A **full-featured, CppAD-compatible evaluator** for ``.dml`` files that
use breakpoint tables, gridded N-D linear interpolation, and
``<piecewise>`` branching. It is the engine behind both the aerodynamic
and propulsion models.

*Additional MathML operators beyond* :ref:`daveml_reader`:
``<abs>``, ``<cos>``, ``<sin>``,
comparison operators (``<lt>``, ``<gt>``, ``<leq>``, ``<geq>``, ``<eq>``
— each returns 1.0 or 0.0),
and branching via ``<piecewise>/<piece>/<otherwise>``.

*Evaluation order* is established once at construction time using
**Kahn's topological sort** of the variable dependency graph, ensuring
each calculated output is evaluated after all its inputs are ready.

*CppAD strategy* — the class is templated on a scalar type ``S``.
Branching conditions are always extracted as plain ``double`` via
``CppAD::Value()``, so branch selection never enters the AD tape.
Arithmetic inside the selected branch is fully carried out in ``S``,
giving correct first derivatives everywhere except at discontinuous
switching boundaries (where derivatives are undefined in any case).

.. code-block:: cpp

   #include "Aetherion/Serialization/DAVEML/DAVEMLAeroModel.h"

   DAVEMLAeroModel model("data/F16_S119_source/F16_aero.dml");

   DAVEMLAeroModel::Inputs<double> in;
   in.vt_fps    = 300.0;   // ft/s
   in.alpha_deg =   5.0;   // deg
   in.beta_deg  =   0.0;
   in.p_rps = in.q_rps = in.r_rps = 0.0;
   in.el_deg = in.ail_deg = in.rdr_deg = 0.0;

   auto out = model.evaluate(in);
   // out.cx, out.cy, out.cz, out.cl, out.cm, out.cn

   // AD path — identical call, different scalar type:
   DAVEMLAeroModel::Inputs<CppAD::AD<double>> in_ad;
   // ... fill in_ad ...
   auto out_ad = model.evaluate(in_ad);

Lower-level access is available through ``evaluateRaw``, which accepts
and returns a ``std::unordered_map<std::string, S>`` keyed by DAVE-ML
variable identifiers.

.. _daveml_prop:

DAVEMLPropModel
~~~~~~~~~~~~~~~

A thin **propulsion wrapper** around :ref:`daveml_aero` for
``F16_prop.dml``. It converts the model's native outputs (lbf, ft·lbf)
to SI (N, N·m) before returning them.

.. code-block:: cpp

   #include "Aetherion/Serialization/DAVEML/DAVEMLPropModel.h"

   DAVEMLPropModel prop("data/F16_S119_source/F16_prop.dml");

   DAVEMLPropModel::Inputs<double> pin;
   pin.pwr_pct = 50.0;   // military (dry) throttle
   pin.alt_ft  =  0.0;   // sea level
   pin.mach    =  0.0;

   auto pout = prop.evaluate(pin);
   // pout.fx_N  — forward thrust force [N]
   // pout.fy_N, pout.fz_N — lateral/normal thrust [N]
   // pout.mx_Nm, pout.my_Nm, pout.mz_Nm — thrust moments [N·m]

   // Scalar convenience helper (double only):
   double T_lbf = prop.thrustX_lbf(50.0, 0.0, 0.0);

Unit conversion constants:

.. list-table::
   :header-rows: 1
   :widths: 25 25 50

   * - Native
     - SI
     - Factor
   * - lbf
     - N
     - 4.448 221 615 260 751
   * - ft·lbf
     - N·m
     - 1.355 817 948 329 279

.. _load_inertia:

LoadInertiaFromDAVEML
~~~~~~~~~~~~~~~~~~~~~

A convenience **free function** that reads ``F16_inertia.dml`` and
returns a populated ``RigidBody::InertialParameters`` struct with all
values converted to SI.

.. code-block:: cpp

   #include "Aetherion/Serialization/DAVEML/LoadInertiaFromDAVEML.h"

   // Default CG position (35 % MAC):
   auto I = LoadInertiaFromDAVEML(
       "data/F16_S119_source/F16_inertia.dml");

   // Override CG to 30 % MAC (5 % forward of reference):
   auto I_fwd = LoadInertiaFromDAVEML(
       "data/F16_S119_source/F16_inertia.dml",
       {{"CG_PCT_MAC", 30.0}});

The variable-ID-to-struct-member mapping is:

.. list-table::
   :header-rows: 1
   :widths: 15 20 20 45

   * - VarID
     - Field
     - Native units
     - Notes
   * - ``XMASS``
     - ``mass``
     - slug
     - Total aircraft mass
   * - ``XIXX``
     - ``Ixx``
     - slug·ft²
     - Roll moment of inertia
   * - ``XIYY``
     - ``Iyy``
     - slug·ft²
     - Pitch moment of inertia
   * - ``XIZZ``
     - ``Izz``
     - slug·ft²
     - Yaw moment of inertia
   * - ``XIZX``
     - ``Ixz``
     - slug·ft²
     - XZ cross-product of inertia
   * - ``XIXY``
     - ``Ixy``
     - slug·ft²
     - XY cross-product (zero for F-16)
   * - ``XIYZ``
     - ``Iyz``
     - slug·ft²
     - YZ cross-product (zero for F-16)
   * - ``DXCG``
     - ``dx``
     - ft
     - Longitudinal CG offset from moment reference (calculated)
   * - ``DYCG``
     - ``dy``
     - ft
     - Lateral CG offset (zero for F-16)
   * - ``DZCG``
     - ``dz``
     - ft
     - Vertical CG offset (zero for F-16)


F-16 DAVE-ML Data Files
------------------------

All five ``.dml`` files reside in ``data/F16_S119_source/``.  They
implement the F-16 model described in Stevens & Lewis [SL03]_ as adapted
and corrected by Morelli (NASA TM-2003-212145 [Mo03]_), encoded to
DAVE-ML 2.0 conformance.  The dataset was obtained from the NASA NESC
Academy Flight Simulation Resources page.

.. _dml_aero:

F16_aero.dml — Aerodynamic Coefficients
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**Purpose.** Computes the six non-dimensional aerodynamic coefficients
used to form body-axis forces and moments via the standard aerodynamic
expansion

.. math::

   \begin{aligned}
   F_X &= \bar{q}\,S\,C_X, &
   F_Y &= \bar{q}\,S\,C_Y, &
   F_Z &= \bar{q}\,S\,C_Z, \\
   L   &= \bar{q}\,S\,b\,C_l, &
   M   &= \bar{q}\,S\,\bar{c}\,C_m, &
   N   &= \bar{q}\,S\,b\,C_n,
   \end{aligned}

where :math:`\bar{q} = \tfrac{1}{2}\rho V_T^2` is dynamic pressure.

**Reference geometry** (embedded in the file):

.. list-table::
   :header-rows: 0
   :widths: 40 20 40

   * - Wing reference area :math:`S`
     - 300.0 ft²
     - 27.87 m²
   * - Mean aerodynamic chord :math:`\bar{c}`
     - 11.32 ft
     - 3.450 m
   * - Wing span :math:`b`
     - 30.0 ft
     - 9.144 m

**Inputs:**

.. list-table::
   :header-rows: 1
   :widths: 15 15 70

   * - VarID
     - Units
     - Description
   * - ``vt``
     - ft/s
     - True airspeed :math:`V_T`
   * - ``alpha``
     - deg
     - Angle of attack :math:`\alpha`
   * - ``beta``
     - deg
     - Angle of sideslip :math:`\beta`
   * - ``p``
     - rad/s
     - Body roll rate
   * - ``q``
     - rad/s
     - Body pitch rate
   * - ``r``
     - rad/s
     - Body yaw rate
   * - ``el``
     - deg
     - Elevator deflection (+ trailing-edge down)
   * - ``ail``
     - deg
     - Aileron deflection (+ right trailing-edge down)
   * - ``rdr``
     - deg
     - Rudder deflection (+ trailing-edge left)

**Outputs:**

.. list-table::
   :header-rows: 1
   :widths: 15 15 70

   * - VarID
     - Sign
     - Description
   * - ``cx``
     - + forward
     - Body-axis X-force coefficient
   * - ``cy``
     - + starboard
     - Body-axis Y-force coefficient
   * - ``cz``
     - + down
     - Body-axis Z-force coefficient
   * - ``cl``
     - + right-wing-down
     - Rolling-moment coefficient
   * - ``cm``
     - + nose-up
     - Pitching-moment coefficient
   * - ``cn``
     - + nose-right
     - Yawing-moment coefficient

**Gridded tables.** The database contains multi-dimensional breakpoint
sets spanning angle of attack, Mach number, sideslip angle, and control
deflection, as defined in Morelli's adaptation of the Stevens & Lewis
nonlinear model.  Interpolation is bilinear in each table dimension;
extrapolation clamps to the nearest breakpoint.

**Embedded check-case** (nominal, all rates and deflections zero):

.. list-table::
   :header-rows: 1
   :widths: 30 70

   * - Input
     - Value
   * - ``vt``
     - 300 ft/s
   * - ``alpha``
     - 5 deg
   * - All others
     - 0

.. list-table::
   :header-rows: 1
   :widths: 15 25 60

   * - Output
     - Expected value
     - Tolerance
   * - ``cx``
     - −0.004
     - ±1 × 10⁻⁴
   * - ``cy``
     - 0.000
     - ±1 × 10⁻⁴
   * - ``cz``
     - −0.416
     - ±1 × 10⁻⁴
   * - ``cl``
     - 0.000
     - ±1 × 10⁻⁴
   * - ``cm``
     - −0.005
     - ±1 × 10⁻⁴
   * - ``cn``
     - 0.000
     - ±1 × 10⁻⁴

**Revision history.** The file has undergone 14 revisions (A–P,
2003–2013).  The most recent, revision P (2013-10-21), corrected
variable-name references inside the embedded check-data blocks.
Earlier revisions updated the DAVE-ML namespace from 1.9 to 2.0 and
corrected minor transcription errors in the breakpoint vectors.

**C++ class:** ``DAVEMLAeroModel`` (see :ref:`daveml_aero`).

.. _dml_prop:

F16_prop.dml — Propulsion Model
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**Purpose.** Computes engine thrust forces and moments as a function of
power lever angle, altitude, and Mach number.  The engine is modelled as
a GE F100 turbofan with a two-regime thrust law: *idle-to-military* and
*military-to-maximum (afterburner)*.

**Inputs:**

.. list-table::
   :header-rows: 1
   :widths: 15 15 70

   * - VarID
     - Units
     - Description
   * - ``PWR``
     - %
     - Power lever angle (0 = idle, 50 = military/dry, 100 = max/afterburner)
   * - ``ALT``
     - ft
     - Altitude above mean sea level
   * - ``RMACH``
     - —
     - Flight Mach number

**Outputs:**

.. list-table::
   :header-rows: 1
   :widths: 15 15 70

   * - VarID
     - Units
     - Description
   * - ``FEX``
     - lbf
     - Body-axis X thrust (positive forward)
   * - ``FEY``
     - lbf
     - Body-axis Y thrust (positive starboard); zero for this model
   * - ``FEZ``
     - lbf
     - Body-axis Z thrust (positive down); zero for this model
   * - ``TEL``
     - ft·lbf
     - Rolling thrust moment (positive right-wing-down)
   * - ``TEM``
     - ft·lbf
     - Pitching thrust moment (positive nose-up)
   * - ``TEN``
     - ft·lbf
     - Yawing thrust moment (positive nose-right)

**Breakpoints.** The thrust tables are indexed by two independent
variables:

.. list-table::
   :header-rows: 1
   :widths: 30 70

   * - Variable
     - Breakpoints
   * - Mach
     - 0.0, 0.2, 0.4, 0.6, 0.8, 1.0
   * - Altitude (ft)
     - 0, 10 000, 20 000, 30 000, 40 000, 50 000

**Gridded tables** (6 × 6 each, row = Mach, column = altitude):

``T_IDLE_table`` — idle thrust (``PWR`` = 0 %):

.. code-block:: none

   Mach\Alt:    0      10k    20k    30k    40k    50k    [lbf]
   0.0:      1 060    670    880    590    960    225
   0.2:        635    425    690    445    725    170
   0.4:        60     25    345    250    400    -  (extrapolated)
   …

``T_MIL_table`` — military (dry) thrust (``PWR`` = 50 %):

.. code-block:: none

   Mach\Alt:    0      10k    20k    30k    40k    50k    [lbf]
   0.0:     12 680   9 150   6 200   3 950   2 450   1 400
   0.2:     12 680   9 150   6 313   4 113   2 600   1 400
   …

``T_MAX_table`` — maximum/afterburner thrust (``PWR`` = 100 %):

.. code-block:: none

   Mach\Alt:    0      10k    20k    30k    40k    50k    [lbf]
   0.0:     20 000  15 000  10 800   7 165   4 000   2 500
   0.6:     22 700  16 800  12 150   8 600   5 250   2 900
   1.0:     28 885  20 950  14 700   9 500   5 700   3 570
   …

**Thrust interpolation law.** Given the bilinearly interpolated idle,
military, and maximum thrust values at the current Mach and altitude:

.. math::

   F_X =
   \begin{cases}
     T_\text{idle} +
       \dfrac{\mathrm{PWR}}{50}
       \bigl(T_\text{mil} - T_\text{idle}\bigr)
     & \text{if } \mathrm{PWR} \le 50, \\[10pt]
     T_\text{mil} +
       \dfrac{\mathrm{PWR} - 50}{50}
       \bigl(T_\text{max} - T_\text{mil}\bigr)
     & \text{if } \mathrm{PWR} > 50.
   \end{cases}

**Check-cases:**

.. list-table::
   :header-rows: 1
   :widths: 15 15 15 20 35

   * - PWR (%)
     - ALT (ft)
     - Mach
     - Expected ``FEX`` (lbf)
     - Notes
   * - 0
     - 0
     - 0.0
     - 1 060
     - Idle, sea level, static
   * - 50
     - 0
     - 0.0
     - 12 680
     - Military, sea level, static
   * - 100
     - 0
     - 0.0
     - 20 000
     - Max, sea level, static
   * - 100
     - 0
     - 1.0
     - 28 885
     - Max, sea level, Mach 1
   * - 100
     - 50 000
     - 1.0
     - 5 057
     - Max, high altitude, Mach 1

**C++ class:** ``DAVEMLPropModel`` (see :ref:`daveml_prop`).

.. _dml_inertia:

F16_inertia.dml — Inertial Properties
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**Purpose.** Defines the mass, rotational inertia tensor, and
center-of-gravity offset of the F-16 at a user-specified longitudinal
CG position.  All values correspond to the nominal clean configuration
from Stevens & Lewis [SL03]_.

**Input:**

.. list-table::
   :header-rows: 1
   :widths: 20 10 70

   * - VarID
     - Units
     - Description
   * - ``CG_PCT_MAC``
     - %
     - Longitudinal CG position as a percentage of the mean aerodynamic chord,
       measured aft of the leading edge. Default: 35.0 (moment reference center).

**Constant outputs:**

.. list-table::
   :header-rows: 1
   :widths: 15 20 20 45

   * - VarID
     - Value
     - Units
     - Description
   * - ``XMASS``
     - 637.1595
     - slug
     - Total aircraft mass (≈ 20 500 lbm ≈ 9 295 kg)
   * - ``XIXX``
     - 9 496.0
     - slug·ft²
     - Roll moment of inertia :math:`I_{xx}`
   * - ``XIYY``
     - 55 814.0
     - slug·ft²
     - Pitch moment of inertia :math:`I_{yy}`
   * - ``XIZZ``
     - 63 100.0
     - slug·ft²
     - Yaw moment of inertia :math:`I_{zz}`
   * - ``XIZX``
     - 982.0
     - slug·ft²
     - XZ cross-product of inertia :math:`I_{xz}`
   * - ``XIXY``
     - 0.0
     - slug·ft²
     - XY cross-product :math:`I_{xy}` (lateral symmetry)
   * - ``XIYZ``
     - 0.0
     - slug·ft²
     - YZ cross-product :math:`I_{yz}` (lateral symmetry)
   * - ``DYCG``
     - 0.0
     - ft
     - Lateral CG offset (lateral symmetry)
   * - ``DZCG``
     - 0.0
     - ft
     - Vertical CG offset

**Calculated output:**

The longitudinal CG offset from the moment reference center (positive aft):

.. math::

   \mathrm{DXCG}
   = 0.01 \times \bar{c} \times
     \bigl(35 - \mathrm{CG\_PCT\_MAC}\bigr)
   \quad [\text{ft}],

where :math:`\bar{c} = 11.32` ft.  At the reference CG (35 % MAC) the
offset is zero; a CG forward of 35 % MAC produces a positive (aft-of-CG)
offset.

Example: ``CG_PCT_MAC = 30`` → ``DXCG = 0.01 × 11.32 × 5 = 0.566 ft``.

**C++ function:** ``LoadInertiaFromDAVEML`` (see :ref:`load_inertia`).

.. _dml_control:

F16_control.dml — Flight Control Laws
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**Purpose.** Implements an F-16 flight control system comprising an
inner-loop **Stability Augmentation System (SAS)** based on a
pre-computed LQR gain matrix and an outer-loop **autopilot** with
altitude hold, heading hold, and airspeed command modes.

**Design trim condition:**

.. list-table::
   :header-rows: 0
   :widths: 40 60

   * - Altitude
     - 10 013 ft MSL
   * - True airspeed
     - 565.7 ft/s (≈ 287.8 kt EAS)
   * - Mach
     - 0.525
   * - Trim angle of attack :math:`\alpha_0`
     - 2.654 deg
   * - Trim pitch angle :math:`\theta_0`
     - 2.654 deg
   * - Throttle trim
     - 13.90 %
   * - Longitudinal stick trim
     - 12.96 %

**Inputs:**

.. list-table::
   :header-rows: 1
   :widths: 20 15 65

   * - VarID
     - Units
     - Description
   * - ``throttle``
     - frac [0–1]
     - Pilot throttle command
   * - ``longStk``
     - frac [−1…+1]
     - Pilot longitudinal (pitch) stick command
   * - ``latStk``
     - frac [−1…+1]
     - Pilot lateral (roll) stick command
   * - ``pedal``
     - frac [−1…+1]
     - Pilot rudder pedal command
   * - ``SAS_on``
     - bool (0/1)
     - Enable stability augmentation
   * - ``AP_on``
     - bool (0/1)
     - Enable autopilot
   * - ``airspeed``
     - ft/s
     - Current true airspeed
   * - ``alpha``
     - deg
     - Current angle of attack
   * - ``beta``
     - deg
     - Current sideslip angle
   * - ``phi``, ``theta``, ``psi``
     - deg
     - Current Euler angles (roll, pitch, yaw)
   * - ``p``, ``q``, ``r``
     - rad/s
     - Current body rates
   * - ``alt``
     - ft
     - Current altitude MSL
   * - ``altCmd``
     - ft
     - Autopilot altitude command
   * - ``airspeedCmd``
     - ft/s
     - Autopilot airspeed command
   * - ``latOffset``
     - ft
     - Lateral offset from desired track
   * - ``baseCourse``
     - deg
     - Desired base course heading

**Outputs:**

.. list-table::
   :header-rows: 1
   :widths: 20 15 65

   * - VarID
     - Units
     - Description
   * - ``el``
     - deg
     - Elevator deflection command
   * - ``ail``
     - deg
     - Aileron deflection command
   * - ``rdr``
     - deg
     - Rudder deflection command
   * - ``throttleCmd``
     - %
     - Power lever angle command (0–100)

**Control law structure.**

The SAS inner loop uses pre-computed LQR gain matrices.
The longitudinal loop stabilises the state vector
:math:`[V, \alpha, q, \theta]^\top` and drives two outputs
(elevator, throttle):

.. math::

   \begin{bmatrix} \delta e \\ \delta T \end{bmatrix}
   = -K_\mathrm{lon}
     \begin{bmatrix} \Delta V \\ \Delta\alpha \\ \Delta q \\ \Delta\theta \end{bmatrix},

and the lateral-directional loop stabilises
:math:`[\phi, \beta, p, r]^\top` and drives (aileron, rudder):

.. math::

   \begin{bmatrix} \delta a \\ \delta r \end{bmatrix}
   = -K_\mathrm{latdir}
     \begin{bmatrix} \Delta\phi \\ \Delta\beta \\ \Delta p \\ \Delta r \end{bmatrix}.

Outer-loop autopilot logic:

- **Altitude hold** — altitude error feeds a pitch-angle command that
  is added to the LQR trim pitch demand.
- **Heading hold** — lateral track error and course-angle error are
  combined into a roll-angle command.
- **Mode switching** — ``<piecewise>`` blocks disable the pilot stick
  when the autopilot is active (``AP_on > 0.5``); SAS gains are
  similarly defeated if both ``SAS_on`` and ``AP_on`` are false.

**Note.** This file defines the control *law* only; it does not integrate
actuator dynamics.  Actuator rate and position limits must be applied
externally if required.

.. _dml_gnc:

F16_gnc.dml — Guidance, Navigation and Control
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**Purpose.** Extends :ref:`dml_control` with a **circumnavigator
guidance** layer that commands the aircraft to fly a 3 nautical-mile
radius circle around a fixed geographic waypoint.

Two selectable navigation modes are provided:

.. list-table::
   :header-rows: 1
   :widths: 10 90

   * - Mode
     - Target waypoint
   * - 1
     - Geographic North Pole
   * - 2
     - Equator / International Date Line intersection (0° N, 180° E)

**Additional inputs** (beyond :ref:`dml_control`):

.. list-table::
   :header-rows: 1
   :widths: 20 15 65

   * - VarID
     - Units
     - Description
   * - ``latitude``
     - deg
     - Current geodetic latitude
   * - ``longitude``
     - deg
     - Current geodetic longitude
   * - ``navMode``
     - integer
     - Circumnavigator mode selector (1 or 2)

**Guidance geometry.** The file computes, in a local planar
approximation:

1. North and East offsets from the target waypoint (ft).
2. Slant range from waypoint: :math:`R = \sqrt{N^2 + E^2}`.
3. Desired track bearing: :math:`\psi_d = \mathrm{atan2}(E, N)`.
4. Lateral offset from the 3 nm circle: :math:`\Delta_\perp = R - 3\,\mathrm{nm}`.

These quantities drive the lateral-offset and base-course inputs of the
inner control law from :ref:`dml_control`.  All other inputs and outputs
are identical to that file.


.. _daveml_tests:

Validation and Test Suite
--------------------------

Unit tests for the DAVE-ML subsystem live in ``tests/Serialization/`` and
are written using the Catch2 framework.  All tests are parameterised to
skip gracefully when the data files are not found, so the test suite
remains usable in environments where ``data/F16_S119_source/`` is absent.

**test_DAVEMLReader.cpp**

- All inertia variable IDs are present in the file.
- Constant values match Stevens & Lewis Table B.1 (double precision).
- ``getValueSI`` applies the correct conversion factors.
- ``DXCG`` evaluates to zero at the reference CG (35 % MAC).
- ``setInput("CG_PCT_MAC", 30.0)`` causes ``DXCG`` to re-evaluate to
  ``0.566 ft`` to within machine precision.
- ``LoadInertiaFromDAVEML`` populates ``InertialParameters`` correctly.

**test_DAVEMLAeroModel.cpp**

- Reference geometry fields match the file constants.
- Nominal check-case (``vt`` = 300 ft/s, ``alpha`` = 5 deg, all others
  zero) produces the expected six coefficients to within ±10⁻⁴.
- The same inputs via ``CppAD::AD<double>`` produce values consistent
  with ``CppAD::Value()`` extraction.
- ``cz`` is negative for positive ``alpha`` throughout the flight
  envelope (physical sanity).

**test_DAVEMLPropModel.cpp**

- Idle, military, and maximum check-case thrust values match the table
  to within ±0.5 lbf.
- Intermediate power settings interpolate monotonically between the
  regime endpoints.
- High-altitude, high-Mach point (PWR = 100 %, ALT = 50 000 ft,
  Mach = 1.0) returns 5 057 lbf.
- ``CppAD::AD<double>`` path returns values consistent with the double
  path.
- Thrust is non-decreasing in ``PWR`` at any fixed (ALT, Mach) point.


References
----------

.. [SL03] B. L. Stevens and F. L. Lewis,
   *Aircraft Control and Simulation*, 2nd ed.
   Wiley, 2003.

.. [Mo03] E. A. Morelli,
   "Global Nonlinear Parametric Modelling with Application to F-16
   Aerodynamics,"
   NASA TM-2003-212145, 2003.

.. [AIAA-S119] AIAA,
   *Dynamic Aerospace Vehicle Exchange Markup Language (DAVE-ML)
   Reference Manual*, AIAA Standard S-119, 2011.
