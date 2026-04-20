.. ------------------------------------------------------------------------------
.. Project: Aetherion
.. Copyright (c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
..
.. SPDX-License-Identifier: MIT
.. License-Filename: LICENSE
.. ------------------------------------------------------------------------------

.. _api:

Aetherion C++ API Reference
============================

This page documents the public C++ API extracted from source headers via
Doxygen and rendered by Breathe.  All templated types are compatible with
both ``double`` and ``CppAD::AD<double>`` unless stated otherwise.

.. contents:: Modules
   :depth: 1
   :local:

Spatial Algebra
---------------

Rigid-body spatial vectors, transforms, and operators following the spatial
vector algebra conventions of Featherstone (2008).

**Storage conventions** (used throughout the library):

* :cpp:class:`Aetherion::Spatial::Twist` — packed 6-vector :math:`[\omega;\,v]`:
  indices 0–2 angular velocity [rad/s], 3–5 linear velocity [m/s].
* :cpp:class:`Aetherion::Spatial::Wrench` — packed 6-vector :math:`[M;\,F]`:
  indices 0–2 moment [N·m], 3–5 force [N] (moment-first, consistent with the
  inner product :math:`P = \mathbf{f}^\top\mathbf{v}`).

Key types and free functions:

* ``Twist<S>`` / ``Wrench<S>`` / ``Momentum<S>`` / ``Inertia<S>`` — fundamental spatial quantities.
* ``skew(v)`` — 3×3 skew-symmetric (cross-product) matrix.
* ``ad(xi)`` / ``ad_star(xi)`` / ``ad_star_times(xi, y)`` — Lie-algebra adjoint operators.
* ``CrossMotion`` / ``CrossForce`` — spatial cross-product operators.
* ``MotionTransformMatrix`` / ``ForceTransformMatrix`` / ``TransformMotion`` / ``TransformForce``
  — 6×6 spatial transforms between frames.
* ``ShiftWrenchToNewPoint`` — re-express a wrench from point P to point Q.
* ``Power`` — instantaneous spatial power :math:`P = \mathbf{f}^\top\mathbf{v}` [W].

.. doxygennamespace:: Aetherion::Spatial
   :content-only:

Lie-Group ODE Solvers
---------------------

SO(3) / SE(3) exponential maps, left Jacobians, and the RKMK family of
structure-preserving integrators.

The RKMK (Runge–Kutta–Munthe-Kaas) framework advances differential equations
that evolve on Lie groups by composing exponential maps rather than naive
Euclidean steps, preserving the manifold structure of the solution at each stage.

Sub-modules:

* ``Aetherion::ODE::RKMK::Lie`` — :math:`\mathrm{SO}(3)` and :math:`\mathrm{SE}(3)`
  exponential/logarithm maps and left Jacobians.
* ``Aetherion::ODE::RKMK::Core`` — Butcher tableaux, stage packing, Newton solver,
  and C++20 concepts constraining integrator template parameters.
* ``Aetherion::ODE::RKMK::Integrators`` — concrete Radau IIA integrators on
  :math:`SE(3)` and the product manifold :math:`SE(3) \times \mathbb{R}^n`.

.. doxygennamespace:: Aetherion::ODE::RKMK::Lie
   :content-only:

.. doxygennamespace:: Aetherion::ODE::RKMK::Core
   :content-only:

.. doxygennamespace:: Aetherion::ODE::RKMK::Integrators
   :content-only:

Flight Dynamics
---------------

State-vector construction, kinematics field, and physics policy interfaces.

**Policy system** — :cpp:class:`Aetherion::RigidBody::VectorField` is templated on
four policy types.  Each policy must satisfy the corresponding C++20 concept:

.. list-table::
   :header-rows: 1
   :widths: 25 35 40

   * - Concept
     - Required signature
     - Built-in policies
   * - ``GravityPolicy``
     - ``Wrench<S> operator()(const SE3<S>&, S mass) const``
     - ``ZeroGravityPolicy``, ``CentralGravityPolicy``, ``J2GravityPolicy``
   * - ``AeroPolicy``
     - ``Wrench<S> operator()(const SE3<S>&, const Matrix<S,6,1>&, S mass, S t) const``
     - ``ZeroAeroPolicy``
   * - ``PropulsionPolicy``
     - same as ``AeroPolicy``
     - ``ZeroPropulsionPolicy``, ``ConstantThrustPolicy``
   * - ``MassPolicy``
     - ``S mdot(S t, S mass) const``
     - ``ConstantMassPolicy``, ``LinearBurnPolicy``

All policies satisfy their concept for both ``S = double`` and
``S = CppAD::AD<double>`` (enforced by ``static_assert`` at definition time).

:cpp:class:`Aetherion::FlightDynamics::KinematicsXiField` implements the
right-trivialised kinematic equation
:math:`\dot{g} = g\,\hat{\xi}` as a stateless identity pass-through
(the body-frame twist *is* the Lie-algebra velocity in this formulation).

.. doxygennamespace:: Aetherion::FlightDynamics
   :content-only:

Rigid Body
----------

The 6-DoF rigid-body simulation layer built on top of the Spatial and
FlightDynamics modules.

**State manifold** :math:`SE(3) \times \mathbb{R}^7`:

* ``g ∈ SE(3)`` — attitude (rotation matrix) and ECI position.
* ``ν_B ∈ ℝ⁶`` — body-frame twist :math:`[\omega_B;\,v_B]` [rad/s | m/s].
* ``m ∈ ℝ`` — current vehicle mass [kg].

Key types:

* :cpp:struct:`Aetherion::RigidBody::State` — full state on :math:`SE(3) \times \mathbb{R}^7`.
* :cpp:struct:`Aetherion::RigidBody::InertialParameters` — mass, inertia tensor about CoG, CoG offset (all in body frame).
* :cpp:class:`Aetherion::RigidBody::VectorField` — Newton-Euler ODE right-hand side, templated on physics policies.
* :cpp:class:`Aetherion::RigidBody::SixDoFStepper` — high-level facade combining kinematics and dynamics into a single ``step()`` call via Radau IIA RKMK.
* :cpp:struct:`Aetherion::RigidBody::StateLayout` — flat 14-element index map for serialised state vectors.

.. doxygennamespace:: Aetherion::RigidBody
   :content-only:

Environment
-----------

Atmospheric models, gravity acceleration models, and WGS-84 geodetic constants.

* **US Standard Atmosphere 1976** — density, pressure, temperature, and speed of
  sound as functions of altitude.  CppAD-friendly.
* **Gravity models** — ``CentralGravity(r, mu)`` (inverse-square) and
  ``J2(r, mu, Re, J2)`` (first zonal harmonic).
* **WGS-84 constants** — ``kSemiMajorAxis_m``, ``kFlattening``, ``kGM_m3_s2``,
  ``kJ2``, ``kEarthAngularVelocity_rad_s`` and others in ``Aetherion::Environment::WGS84``.

.. doxygennamespace:: Aetherion::Environment
   :content-only:

Coordinate Transforms
---------------------

Geodetic ↔ ECEF ↔ ECI conversions, NED frame utilities, and attitude decomposition.

**Frame definitions** used throughout:

* **ECI** (Earth-Centred Inertial) — origin at Earth's centre, axes fixed to
  distant stars; used as the integration frame.
* **ECEF** (Earth-Centred Earth-Fixed) — same origin, rotates with Earth at
  :math:`\dot{\theta}_{ERA}`.
* **NED** (North-East-Down) — local tangent frame at a surface point;
  right-handed with x = North, y = East, z = Down.
* **Body** — x forward, y right, z down.

All functions accept a template scalar ``S`` and are CppAD-friendly.

**Transform chain** (forward — local to inertial):

.. code-block:: text

   Geodetic (φ, λ, h)
       → ECEF  [GeodeticToECEF]
       → ECI   [ECEFToECI, requires θ_ERA]
       → launch state [MakeLaunchStateECI]

**Transform chain** (inverse — inertial to local):

.. code-block:: text

   ECI position
       → ECEF  [ECIToECEF]
       → Geodetic (φ, λ, h)  [ECEFToGeodeticWGS84]
       → NED   [ECEFToNED / ECIToNED]
       → local orientation  [QuaternionToAzZenRollNED]

.. doxygennamespace:: Aetherion::Coordinate
   :content-only:

Serialization
-------------

JSON (de)serialization for all configuration and state types, built on
`nlohmann/json <https://github.com/nlohmann/json>`_.

* ``LoadConfig(filename)`` / ``LoadConfig(json)`` — load a
  :cpp:struct:`Aetherion::RigidBody::Config` from a JSON file or a pre-parsed
  ``nlohmann::json`` object.
* ``LoadConfigFromString(str)`` — parse and validate a JSON string directly
  (throws ``std::runtime_error`` on empty input, parse failure, or schema mismatch).
* ``from_json`` / ``to_json`` specialisations for all configuration structs:
  ``GeodeticPoseNED``, ``VelocityNED``, ``BodyRates``, ``InertialParameters``,
  ``AerodynamicParameters``, and ``Config``.

.. doxygennamespace:: Aetherion::Serialization
   :content-only:

Simulation Framework
--------------------

Application base class, argument parser, simulator interface, and telemetry types.

**Template Method pattern** — :cpp:class:`Aetherion::Simulation::Application` defines
the simulation loop in ``run()`` and exposes the following virtual hooks for
subclasses to override:

.. list-table::
   :header-rows: 1
   :widths: 35 65

   * - Hook
     - Responsibility
   * - ``prepareSimulation()``
     - Allocate and initialise all simulation objects before the loop starts.
   * - ``writeInitialSnapshot(ofstream&)``
     - Write the :math:`t=0` state to the output CSV.
   * - ``stepAndRecord(ofstream&, double t, bool doWrite)``
     - Advance by one time step; write to CSV only when ``doWrite`` is true.
   * - ``logFinalSummary()``
     - Print end-of-simulation diagnostics.

:cpp:class:`Aetherion::Simulation::ArgumentParser` parses ``--timeStep``,
``--startTime``, ``--endTime``, ``--inputFileName``, ``--outputFileName``, and
``--writeInterval`` from ``argv``, throwing ``std::invalid_argument`` on
unknown flags, missing values, or constraint violations.

.. doxygennamespace:: Aetherion::Simulation
   :content-only:
