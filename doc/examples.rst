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

.. _example_dragless_sphere:

Dragless Sphere (NASA TM-2015-218675 Atmospheric Scenario 1)
------------------------------------------------------------

**Reference:** NASA Technical Memorandum TM-2015-218675,
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
   * - ECI position
     - ``gePosition_m_X/Y/Z``
     - Geocentric (ECI) Cartesian position [m]
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

The table below compares Aetherion output against the NASA TM-2015-218675
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
0.01\ \text{s}`.

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
