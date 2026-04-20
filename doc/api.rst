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
Doxygen and rendered by Breathe.

.. contents:: Modules
   :depth: 1
   :local:

Spatial Algebra
---------------

Rigid-body spatial vectors (twists, wrenches), transforms, and operators
following the spatial vector algebra conventions of Featherstone.

.. doxygennamespace:: Aetherion::Spatial
   :content-only:

Lie-Group ODE Solvers
---------------------

SO(3) / SE(3) exponential maps, left Jacobians, and the RKMK family of
structure-preserving integrators.

.. doxygennamespace:: Aetherion::ODE::RKMK::Lie
   :content-only:

.. doxygennamespace:: Aetherion::ODE::RKMK::Core
   :content-only:

.. doxygennamespace:: Aetherion::ODE::RKMK::Integrators
   :content-only:

Flight Dynamics
---------------

State-vector construction, kinematic fields, and physics policy interfaces
(gravity, aero, propulsion, mass).

.. doxygennamespace:: Aetherion::FlightDynamics
   :content-only:

Rigid Body
----------

Vehicle configuration, aerodynamic and gravitational wrench computation,
and the 6-DOF stepper.

.. doxygennamespace:: Aetherion::RigidBody
   :content-only:

Environment
-----------

Atmospheric models, gravity models, and WGS-84 constants.

.. doxygennamespace:: Aetherion::Environment
   :content-only:

Coordinate Transforms
---------------------

Geodetic ↔ ECEF ↔ ECI conversions and NED frame utilities.

.. doxygennamespace:: Aetherion::Coordinate
   :content-only:

Serialization
-------------

JSON (de)serialization for all configuration and state types.

.. doxygennamespace:: Aetherion::Serialization
   :content-only:

Simulation Framework
--------------------

Application base class, argument parser, configuration, and logging.

.. doxygennamespace:: Aetherion::Simulation
   :content-only:
