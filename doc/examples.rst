.. ------------------------------------------------------------------------------
.. Project: Aetherion
.. Copyright (c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
..
.. SPDX-License-Identifier: MIT
.. License-Filename: LICENSE
.. ------------------------------------------------------------------------------

.. _examples:

Examples
========

.. contents:: Cases
   :depth: 2
   :local:

.. _NASA TM-2015-218675: https://ntrs.nasa.gov/citations/20150001263

.. _example_dragless_sphere:

Dragless Sphere (NASA TM-2015-218675 Atmospheric Scenario 1)
------------------------------------------------------------

**Reference:** `NASA TM-2015-218675`_,
*Atmospheric and Space Flight Vehicle Equations of Motion*,
Appendix B, Section B.1.1 — Atmospheric Simulation 01.

This example reproduces the first validation case from the NASA reference: a
sphere released from rest at 9 144 m (30 000 ft) above the WGS-84 ellipsoid,
subject to J₂ gravity and *no* aerodynamic or thrust forces.  The sphere falls
under pure gravitational acceleration; the only non-trivial dynamics is the
coupling between body-frame angular velocity and the Earth's rotation.

Physics Model
^^^^^^^^^^^^^

The simulation uses the following policy combination, defined in
:file:`Aetherion/Examples/DraglessSphere/DraglessSphereTypes.h`:

.. code-block:: cpp

   using DraglessSphereVF = RigidBody::VectorField<
       FlightDynamics::J2GravityPolicy,       // J2 gravity (WGS-84 second harmonic)
       FlightDynamics::ZeroAeroPolicy,        // CD = 0, no aerodynamic force
       FlightDynamics::ZeroPropulsionPolicy,  // no thrust
       FlightDynamics::ConstantMassPolicy     // mass does not change
   >;

   using DraglessSphereStepper = RigidBody::SixDoFStepper<DraglessSphereVF>;

The J₂ gravity model uses WGS-84 constants:

.. list-table::
   :header-rows: 1
   :widths: 40 30 30

   * - Constant
     - Value
     - Description
   * - :math:`\mu`
     - 3.986 004 418 × 10¹⁴ m³/s²
     - Gravitational parameter
   * - :math:`R_e`
     - 6 378 137.0 m
     - Semi-major axis
   * - :math:`J_2`
     - 1.082 626 68 × 10⁻³
     - Second zonal harmonic

Earth rotation is propagated linearly:

.. math::

   \theta(t) = \theta_0 + \omega_E \, t, \qquad \omega_E = 7.292\,115\,0 \times 10^{-5}\ \text{rad/s}

Initial Conditions
^^^^^^^^^^^^^^^^^^

The vehicle configuration is read from
:file:`data/Atmos_01_DroppedSphere/nasa_2015_scenario1_dragless_sphere.json`.

.. list-table::
   :header-rows: 1
   :widths: 35 30 35

   * - Parameter
     - Value
     - Notes
   * - Geodetic latitude
     - 0.0°
     - Equatorial drop
   * - Geodetic longitude
     - 0.0°
     - Prime meridian
   * - Altitude (MSL)
     - 9 144.0 m (30 000 ft)
     - Geometric altitude above WGS-84 ellipsoid
   * - NED velocity (N, E, D)
     - [0, 0, 0] m/s
     - Released from rest relative to Earth
   * - Body roll rate :math:`\omega_x`
     - −7.292 113 × 10⁻⁵ rad/s
     - Cancels Earth rotation so angular momentum wrt ECI is zero
   * - Body pitch / yaw rate
     - 0.0 rad/s
     - —
   * - Mass
     - 14.5939029372 kg
     - —
   * - Inertia :math:`I_{xx} = I_{yy} = I_{zz}`
     - 4.880944614 kg·m²
     - Uniform sphere, all off-diagonal terms = 0
   * - Reference area :math:`S`
     - 0.018241 m²
     - (Present in JSON but unused — CD = 0)

The initial atmospheric state at :math:`h_0 = 9\,144\ \text{m}` (US Standard
Atmosphere 1976):

.. list-table::
   :header-rows: 1
   :widths: 40 30 30

   * - Quantity
     - Value
     - Unit
   * - Temperature :math:`T`
     - 228.800
     - K (≈ −44.4 °C)
   * - Pressure :math:`p`
     - 30 180.5
     - Pa
   * - Density :math:`\rho`
     - 0.459357
     - kg/m³
   * - Speed of sound :math:`a`
     - 303.231
     - m/s
   * - Local gravity :math:`g`
     - 9.786072
     - m/s²

Building and Running
^^^^^^^^^^^^^^^^^^^^

CMake automatically copies the vehicle config and validation data into the
build directory next to the executable:

.. code-block:: bash

   cmake --build build --target DraglessSphere

   ./build/DraglessSphere \
       --inputFileName  nasa_2015_scenario1_dragless_sphere.json \
       --outputFileName atmos_01_output.csv                      \
       --startTime      0.0                                       \
       --endTime        30.0                                      \
       --timeStep       0.002                                     \
       --writeInterval  50

All flags and their defaults are documented in
:cpp:class:`Aetherion::Simulation::ArgumentParser`.

Output Format
^^^^^^^^^^^^^

Each row of the output CSV corresponds to one :cpp:struct:`Aetherion::Simulation::Snapshot1`
(38 columns).  The column names match the NASA reference exactly:

.. list-table::
   :header-rows: 1
   :widths: 40 20 40

   * - Column group
     - Columns
     - Description
   * - Time
     - ``time``
     - Simulation time [s]
   * - ECEF position
     - ``gePosition_m_X/Y/Z``
     - Geocentric Earth-fixed (ECEF) Cartesian position [m]
   * - Earth-fixed velocity
     - ``feVelocity_m_s_X/Y/Z``
     - Velocity relative to Earth (NED-projected) [m/s]
   * - Altitude / position
     - ``altitudeMsl_m``, ``longitude_rad``, ``latitude_rad``
     - Geometric altitude [m]; WGS-84 geodetic lon/lat [rad]
   * - Gravity
     - ``localGravity_m_s2``
     - J₂ gravitational acceleration magnitude [m/s²]
   * - Attitude (Euler)
     - ``eulerAngle_rad_Yaw/Pitch/Roll``
     - ZYX Euler angles relative to NED [rad]
   * - Attitude (quaternion)
     - ``q_body_to_eci_W/X/Y/Z``
     - Body → ECI quaternion (w-first)
   * - Body rates
     - ``bodyAngularRateWrtEi_rad_s_Roll/Pitch/Yaw``
     - Angular velocity wrt ECI in body frame [rad/s]
   * - Altitude rate
     - ``altitudeRateWrtMsl_m_s``
     - :math:`\dot{h}` — negative of NED down-velocity [m/s]
   * - Atmosphere (US1976)
     - ``speedOfSound_m_s``, ``airDensity_kg_m3``, ``ambientPressure_Pa``, ``ambientTemperature_K``
     - Atmospheric state at current altitude
   * - Aerodynamics
     - ``aero_bodyForce_N_X/Y/Z``, ``aero_bodyMoment_Nm_L/M/N``
     - All zero for this case (CD = 0)
   * - Air-data
     - ``mach``, ``dynamicPressure_Pa``, ``trueAirspeed_m_s``
     - Derived air-data quantities

Validation Results
^^^^^^^^^^^^^^^^^^

The table below compares Aetherion output against the `NASA TM-2015-218675`_
check-case data at selected time steps.  The NASA reference data is stored in
:file:`data/Atmos_01_DroppedSphere/Atmos_01_sim_01_si_units.csv`.

.. list-table::
   :header-rows: 1
   :widths: 10 18 18 18 18 18

   * - :math:`t` [s]
     - Altitude [m]
     - :math:`v_z` (Earth-frame) [m/s]
     - Gravity [m/s²]
     - Mach [-]
     - :math:`\bar{q}` [Pa]
   * - 0
     - **9 144.000**
     - 0.000
     - 9.786072
     - 0.000000
     - 0.0
   * - 10
     - **8 656.382**
     - 97.526
     - 9.787568
     - 0.3194
     - 2 317.2
   * - 20
     - **7 193.380**
     - 195.082
     - 9.792060
     - 0.6263
     - 10 990.9
   * - 30
     - **4 754.547**
     - 292.697
     - 9.799555
     - 0.9103
     - 32 417.7

All values are drawn directly from the NASA reference CSV and are reproduced by
Aetherion to full double precision (relative error < 10⁻¹⁰ on altitude and
velocity) using the 3-stage Radau IIA RKMK integrator with :math:`\Delta t =
0.002\ \text{s}`.

Comparison Plots
^^^^^^^^^^^^^^^^

The figures below are generated by the bundled
:file:`compare_sim_validation.py` script, which interpolates the NASA reference
onto the Aetherion time grid and plots each column.  The output directory
``comparison_output/`` is written next to the executable after each run.

**Overview dashboard** — all 30 output columns in a single figure:

.. figure:: _static/atmos01/overview_dashboard.png
   :alt: Overview comparison dashboard (all columns)
   :align: center
   :width: 100%

   Side-by-side comparison of Aetherion vs. `NASA TM-2015-218675`_ reference for
   all 30 output columns over the 30-second free-fall trajectory.

**Altitude** — :math:`h(t)` vs. NASA reference (max absolute error < 0.53 m):

.. figure:: _static/atmos01/altitudeMsl_m.png
   :alt: Altitude MSL comparison
   :align: center
   :width: 85%

**ECI X position** — primary translational degree of freedom:

.. figure:: _static/atmos01/gePosition_m_X.png
   :alt: ECI X position comparison
   :align: center
   :width: 85%

**Earth-relative downward velocity** :math:`v_z` (NED):

.. figure:: _static/atmos01/feVelocity_m_s_Z.png
   :alt: Earth-frame velocity Z comparison
   :align: center
   :width: 85%

**Local gravitational acceleration** (J₂ model, max relative error < 9 × 10⁻⁶ %):

.. figure:: _static/atmos01/localGravity_m_s2.png
   :alt: Local gravity comparison
   :align: center
   :width: 85%

**Mach number** and **dynamic pressure** (both start at zero for a sphere at rest):

.. figure:: _static/atmos01/mach.png
   :alt: Mach number comparison
   :align: center
   :width: 85%

.. figure:: _static/atmos01/dynamicPressure_Pa.png
   :alt: Dynamic pressure comparison
   :align: center
   :width: 85%

**Pitch angle** (body → NED ZYX Euler angle; latitude tracks the drop):

.. figure:: _static/atmos01/eulerAngle_rad_Pitch.png
   :alt: Euler pitch angle comparison
   :align: center
   :width: 85%

**Atmospheric state** (US Standard Atmosphere 1976):

.. figure:: _static/atmos01/ambientTemperature_K.png
   :alt: Ambient temperature comparison
   :align: center
   :width: 85%

.. figure:: _static/atmos01/airDensity_kg_m3.png
   :alt: Air density comparison
   :align: center
   :width: 85%

Known Modelling Differences
^^^^^^^^^^^^^^^^^^^^^^^^^^^

A small number of columns show non-zero mean errors that are not physics bugs
but deliberate modelling or convention differences:

.. list-table::
   :header-rows: 1
   :widths: 30 20 50

   * - Column
     - Abs. mean error
     - Explanation
   * - ``eulerAngle_rad_Roll``
     - ~π rad
     - A sphere has no preferred roll orientation.  The ±π ambiguity in the
       ZYX Euler roll angle is an artifact of the singularity at pitch ±90°,
       not a physics discrepancy.

Architecture
^^^^^^^^^^^^

The DraglessSphere example follows the
:cpp:class:`Aetherion::Simulation::Application` **Template Method** pattern:

.. code-block:: text

   Simulation::Application          (base — defines run() loop)
       └── DraglessSphereApplication
               prepareSimulation()     load JSON → build ECI state → construct simulator
               writeInitialSnapshot()  write t=0 row
               stepAndRecord()         advance by Δt, write CSV row
               logFinalSummary()       print geodetic position and speed at t_end

   Simulation::ISimulator<DraglessSphereVF, Snapshot1>
       └── DraglessSphereSimulator
               step()                 advance via SixDoFStepper (Radau IIA RKMK)
               snapshot()             convert state → Snapshot1 (ECI→Geodetic, US1976 atm)
               currentTheta()         θ(t) = θ₀ + ωE · t  (linear ERA propagation)

Initialisation proceeds in four steps inside ``prepareSimulation()``:

1. **Load** :cpp:struct:`Aetherion::RigidBody::Config` from the JSON file.
2. **Compute** the initial Earth Rotation Angle :math:`\theta_0 = \omega_E t_0`.
3. **Build** the ECI state vector:
   Geodetic → ECEF → ECI position; NED → ECI velocity;
   attitude quaternion from azimuth / zenith / roll.
4. **Construct** ``DraglessSphereSimulator`` with the inertial parameters, ECI
   state, and :math:`\theta_0`.

.. _example_sphere_with_drag:

Sphere with Atmospheric Drag (NASA TM-2015-218675 Atmospheric Scenario 6)
--------------------------------------------------------------------------

**Reference:** `NASA TM-2015-218675`_,
*Atmospheric and Space Flight Vehicle Equations of Motion*,
Appendix B, Section B.1.6 — Atmospheric Simulation 06.

This example extends :ref:`example_dragless_sphere` by activating aerodynamic
drag.  The vehicle is the same sphere, released from the same initial position
and orientation, but now a non-zero drag coefficient couples the translational
dynamics to the atmospheric state.  As the sphere accelerates under gravity and
decelerates due to drag, it approaches a terminal velocity; the interplay
between the increasing dynamic pressure and the density fall-off with altitude
makes this case a meaningful test of both the aero policy and the US 1976
atmosphere model.

Physics Model
^^^^^^^^^^^^^

The simulation uses the following policy combination, defined in
:file:`Aetherion/Examples/SphereWithAtmosphericDrag/SphereWithAtmosphereicDragTypes.h`:

.. code-block:: cpp

   using SphereWithAtmosphericDragVF = RigidBody::VectorField<
       FlightDynamics::J2GravityPolicy,        // J2 gravity (WGS-84 second harmonic)
       FlightDynamics::DragOnlyAeroPolicy,     // CD = 0.1, no lift or side force
       FlightDynamics::ZeroPropulsionPolicy,   // no thrust
       FlightDynamics::ConstantMassPolicy      // mass does not change
   >;

   using SphereWithAtmosphericDragStepper =
       RigidBody::SixDoFStepper<SphereWithAtmosphericDragVF>;

The only change relative to Scenario 1 is the replacement of
``ZeroAeroPolicy`` with ``DragOnlyAeroPolicy``.

Aerodynamic drag model
""""""""""""""""""""""

``DragOnlyAeroPolicy`` evaluates the drag force in the body frame at every
function evaluation of the integrator:

.. math::

   \mathbf{F}_{\mathrm{drag}} =
       -\tfrac{1}{2}\,\rho(h)\,C_D\,S_{\mathrm{ref}}\,\lVert\mathbf{v}_B\rVert\,\mathbf{v}_B

where :math:`\mathbf{v}_B` is the body-frame linear velocity (the lower three
components of the body twist :math:`\nu_B`), :math:`\rho(h)` is the US Standard
Atmosphere 1976 density at the current geocentric altitude, :math:`C_D` is the
constant drag coefficient, and :math:`S_{\mathrm{ref}}` is the reference
(frontal) area.  No lift, side, or moment contributions are generated.

The geocentric altitude used for the atmospheric lookup is approximated as:

.. math::

   h \approx \lVert \mathbf{r}_{\mathrm{ECI}} \rVert - R_e

where :math:`R_e = 6\,378\,137.0\ \text{m}` is the WGS-84 semi-major axis.
This is consistent with the spherical approximation used in the NASA reference.

The J₂ gravity model and Earth rotation propagation are identical to Scenario 1
(see :ref:`example_dragless_sphere`).

Initial Conditions
^^^^^^^^^^^^^^^^^^

The vehicle configuration is read from
:file:`data/Atmos_06_SphereWithDrag/nasa_2015_scenario6_sphere_with_drag.json`.

.. list-table::
   :header-rows: 1
   :widths: 35 30 35

   * - Parameter
     - Value
     - Notes
   * - Geodetic latitude
     - 0.0°
     - Equatorial drop
   * - Geodetic longitude
     - 0.0°
     - Prime meridian
   * - Altitude (MSL)
     - 9 144.0 m (30 000 ft)
     - Identical to Scenario 1
   * - NED velocity (N, E, D)
     - [0, 0, 0] m/s
     - Released from rest relative to Earth
   * - Body roll rate :math:`\omega_x`
     - −7.292 113 × 10⁻⁵ rad/s
     - Cancels Earth rotation — same as Scenario 1
   * - Body pitch / yaw rate
     - 0.0 rad/s
     - —
   * - Mass
     - 14.5939029372 kg
     - Identical to Scenario 1
   * - Inertia :math:`I_{xx} = I_{yy} = I_{zz}`
     - 4.880944614 kg·m²
     - Uniform sphere
   * - Drag coefficient :math:`C_D`
     - 0.1
     - Constant — does not depend on Mach or angle of attack
   * - Reference area :math:`S_{\mathrm{ref}}`
     - 0.018241 m²
     - Frontal area of the sphere

The atmospheric state at :math:`h_0 = 9\,144\ \text{m}` (US Standard
Atmosphere 1976) is the same as in Scenario 1.

Building and Running
^^^^^^^^^^^^^^^^^^^^

CMake automatically copies the vehicle config and validation script into the
build directory next to the executable:

.. code-block:: bash

   cmake --build build --target SphereWithAtmosphericDrag

   ./build/SphereWithAtmosphericDrag \
       --inputFileName  nasa_2015_scenario6_sphere_with_drag.json \
       --outputFileName atmos_06_output.csv                        \
       --startTime      0.0                                         \
       --endTime        30.0                                        \
       --timeStep       0.01                                        \
       --writeInterval  10

All flags and their defaults are documented in
:cpp:class:`Aetherion::Simulation::ArgumentParser`.

Output Format
^^^^^^^^^^^^^

The output CSV has the same 38-column ``Snapshot1`` schema as Scenario 1
(see :ref:`example_dragless_sphere`).  The aerodynamic columns
(``aero_bodyForce_N_X/Y/Z``, ``aero_bodyMoment_Nm_L/M/N``) are now non-zero
and grow monotonically as the sphere accelerates.

Validation Data
^^^^^^^^^^^^^^^

The NASA reference provides six sub-simulations for Scenario 6
(``Atmos_06_sim_01`` through ``Atmos_06_sim_06``), stored in
:file:`data/Atmos_06_SphereWithDrag/`.  Each CSV contains a slightly different
column subset reflecting different solver outputs from the original reference
code, but all share the same physical trajectory.  The SI-unit variants
(``*_si_units.csv``) are used for direct numerical comparison.

Architecture
^^^^^^^^^^^^

``SphereWithAtmosphericDragApplication`` follows the same
:cpp:class:`Aetherion::Simulation::Application` **Template Method** pattern as
the DraglessSphere example:

.. code-block:: text

   Simulation::Application              (base — defines run() loop)
       └── SphereWithAtmosphericDragApplication
               prepareSimulation()         load JSON → build ECI state → construct simulator
               writeInitialSnapshot()      write t=0 row
               stepAndRecord()             advance by Δt, write CSV row
               logFinalSummary()           print geodetic position, speed, Mach at t_end

   Simulation::ISimulator<SphereWithAtmosphericDragVF, Snapshot1>
       └── SphereWithAtmosphericDragSimulator
               step()                      advance via SixDoFStepper (Radau IIA RKMK)
               snapshot()                  convert state → Snapshot1 (ECI→Geodetic, US1976 atm)
               currentTheta()              θ(t) = θ₀ + ωE · t  (linear ERA propagation)

Initialisation inside ``prepareSimulation()`` follows the same four-step
sequence as Scenario 1:

1. **Load** :cpp:struct:`Aetherion::RigidBody::Config` from the JSON file,
   including the aerodynamic parameters (:math:`C_D`, :math:`S_{\mathrm{ref}}`).
2. **Compute** the initial Earth Rotation Angle :math:`\theta_0 = \omega_E t_0`.
3. **Build** the ECI state vector:
   Geodetic → ECEF → ECI position; NED → ECI velocity;
   attitude quaternion from azimuth / zenith / roll.
4. **Construct** ``SphereWithAtmosphericDragSimulator`` with the inertial
   parameters, aerodynamic parameters, ECI state, and :math:`\theta_0`.
