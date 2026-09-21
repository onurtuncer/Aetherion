.. ------------------------------------------------------------------------------
.. Project: Aetherion
.. Copyright (c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
..
.. SPDX-License-Identifier: MIT
.. License-Filename: LICENSE
.. ------------------------------------------------------------------------------

.. _fmus:

Shipped FMUs and Their Ports
============================

Aetherion ships three FMI 2.0 **Co-Simulation** FMUs.  Each is a self-contained
archive carrying its own Radau IIA RKMK integrator, its DAVE-ML data files under
``resources/``, and a ``modelDescription.xml`` naming every port below.

.. contents:: Units
   :depth: 2
   :local:

The general FMI background — model exchange versus co-simulation, and how an SVA
wrench model maps onto FMU variables — is in :ref:`the FMU appendix <appendix-fmu>`.  This page is
the concrete interface contract: the port names a consumer binds against.

.. list-table::
   :header-rows: 1
   :widths: 22 14 64

   * - Model
     - Role
     - Physics
   * - ``F16Plant``
     - Truth plant
     - F-16 6-DoF, DAVE-ML aero and propulsion decks, J2 gravity, trim solved at
       initialisation.  NASA TM-2015-218675 scenarios 11–16.
   * - ``F16Autopilot``
     - Controller
     - LQR SAS inner loop with altitude, airspeed and heading hold, from the
       NASA LaRC ``F16_control.dml`` gain set; optionally the circumnavigating
       ``F16_gnc.dml`` variant for scenarios 15 and 16.
   * - ``TwoStageRocket``
     - Truth plant
     - Two-stage rocket 6-DoF with variable inertia, propellant burn and stage
       separation.  NASA TM-2015-218675 scenario 17.

Every FMU declares ``canGetAndSetFMUstate`` and ``canSerializeFMUstate``, so a
master may checkpoint and roll back a step.

.. _fmu_version:

Pinning a version floor
-----------------------

Each ``modelDescription.xml`` carries the Aetherion release it was built from in
the FMI ``version`` attribute of ``fmiModelDescription``:

.. code-block:: xml

   <fmiModelDescription fmiVersion="2.0"
                        modelName="F16Plant"
                        version="0.14.0"
                        generationTool="fmu4cpp v0.14.0">
   </fmiModelDescription>

A consumer enforcing a minimum Aetherion version should read that attribute out
of the located ``.fmu`` archive, rather than trusting a CMake package config.
The failure mode worth guarding against is a stale build tree: a package config
reports the version of the source it was configured from, while the ``.fmu`` on
disk may still carry older ports.  The attribute above is written by the same
build step that packages the archive, so the two cannot disagree.

.. note::

   Do not read ``generationTool`` for this.  It names the exporting library
   (``fmu4cpp``), not the model, even though the two version numbers happen to
   coincide because fmu4cpp is built as part of this project.

``specificForce_{x,y,z}_m_s2`` on ``F16Plant`` and ``TwoStageRocket`` first ship
in **0.14.0**.  ``circumnavigate``, ``cmd.circlePoleSW``, ``fb.lat_deg`` and
``fb.lon_deg`` on ``F16Autopilot`` first ship in **0.15.0**; the same release
is the first in which ``F16Plant`` trims against the level-flight apparent
weight and starts wings-level (see :ref:`fmu_f16plant`).

.. _fmu_conventions:

Conventions common to all ports
-------------------------------

**Naming.** Ports are grouped by a dotted prefix: ``out.`` for outputs, ``ctrl.``
for control-surface signals, ``cmd.`` and ``fb.`` for autopilot commands and
feedback, ``solver.`` and ``stg2.`` for tuning.  Parameters carry no prefix.

**Body axes.** ``x`` forward, ``y`` right, ``z`` down — the standard aircraft
body frame, matching the sign convention of the DAVE-ML force and moment
coefficients.  All ``aero_F*``, ``aero_M*`` and ``specificForce_*`` ports share these
axes.

**Navigation frame.** ``out.v_{north,east,down}_m_s`` is NED, Earth-relative;
``out.{yaw,pitch,roll}_rad`` are ZYX Euler angles for the body → NED rotation.

**Body rates.** ``out.{p,q,r}_rad_s`` are with respect to the **ECI** frame, not
the local level frame.  The difference is Earth's rotation rate, 7.29 × 10⁻⁵
rad/s, which matters for a strapdown gyro model and — because it is steady — for
aerodynamic damping: ``F16Plant`` evaluates its damping derivatives at the rate
relative to the air mass, :math:`\omega_{B/\mathrm{ECEF}}`, not at these
outputs.

**Units** are SI in the ``out.`` block, with the unit carried in the port name
suffix.  Initial-condition parameters keep the units of the source NASA scenario
definition (feet, knots, degrees) and are converted internally.

**Parameter timing.** Ports with variability ``fixed`` must be written before
``fmi2ExitInitializationMode``; that call is what runs the trim solve and builds
the initial state.  Only ``stg2.ignition_time_s`` is ``tunable`` and may be
changed between steps.

**Communication step.** Each FMU integrates internally with the master's
communication step unless ``solver.max_step_s`` is set, in which case the step is
subdivided.  For ``F16Plant`` at the scenario-11 trim point, communication steps
from 0.01 s to 0.5 s give trajectories identical to plotting precision; at 1 s
the Radau step visibly degrades, and derived quantities such as ``out.specificForce_*``
degrade with it.  If outputs are only needed at a coarse rate, set
``solver.max_step_s`` rather than slowing the master.

.. _fmu_specific_force:

Specific force: ``out.specificForce_*``
---------------------------------------

``F16Plant`` and ``TwoStageRocket`` publish body-frame **specific force** at the
centre of gravity:

.. math::

   \mathbf{f}_B
   = \frac{\mathbf{F}_\mathrm{aero} + \mathbf{F}_\mathrm{thrust}}{m},

the non-gravitational acceleration an ideal accelerometer at the CG would sense.
This is the truth input an IMU sensor model consumes.

The name reads as an acceleration, so the semantics are worth stating plainly:

- **Gravitation is excluded entirely.**  A vehicle in free fall reads exactly
  zero, not :math:`-g`.  The implementation sums the non-gravitational wrench
  forces and never consults a gravity policy, so there is no gravity term to get
  wrong.  ``out.g_m_s2`` — a mass-attraction magnitude, J2 included — plays no
  part in it.
- It is **not** :math:`R\,(\mathbf{a}_I - \mathbf{g})` with plumb-bob gravity.
  The centrifugal term is something a real strapdown accelerometer genuinely
  senses; subtracting it would double-count.
- The reference point is the **CG**, where :math:`\mathbf{F} = m\,\mathbf{a}`
  holds exactly.  This is true even for ``TwoStageRocket``, which integrates its
  equations of motion about the moment reference centre ``DXCG`` metres aft of
  the CG: the origin choice affects moment bookkeeping, not the force sum.
- An accelerometer mounted a lever arm :math:`\mathbf{r}` off the CG also senses
  the transport terms
  :math:`\dot{\boldsymbol{\omega}} \times \mathbf{r} +
  \boldsymbol{\omega} \times (\boldsymbol{\omega} \times \mathbf{r})`.
  These are **not** applied.  They need :math:`\dot{\boldsymbol{\omega}}`, which
  the plant has and a consumer does not, so off-CG support — if ever wanted —
  belongs on the FMU side of the boundary.

The thrust axis comes from the vehicle's own propulsion policy.  A consumer
forming specific force from ``out.thrust_N`` and ``out.mass_kg`` instead has to
assume where the engine points; ``out.specificForce_*`` removes that assumption, which
matters for any vehicle with an installation angle or more than one engine.

Sanity checks that catch the gravity-inclusion error, in decreasing sharpness:
a gravity-only vehicle reads identically zero; a drag-only vehicle reads
:math:`-\mathrm{drag}/m`, antiparallel to airspeed; a trimmed aircraft in level
flight reads :math:`|\mathbf{f}_B| \approx g` with a negative z channel, since
lift acts up and body z points down.  These are exercised in
``tests/Simulation/test_BodySpecificForce.cpp``.

.. _fmu_f16plant:

F16Plant
--------

F-16 six-degree-of-freedom plant.  ``fmi2ExitInitializationMode`` loads the
DAVE-ML decks, runs a CppAD-backed Newton trim solve at the requested flight
condition, and seeds the integration state from the trim point; the elevator and
throttle inputs are initialised to their trim values, so an open-loop run with
untouched inputs starts in trim.

The trim balances the **apparent** weight of level flight, not the static one:
J2 attraction at ``lat0_deg`` / ``alt0_ft``, less the centripetal acceleration
of a constant-altitude path over the rotating Earth at ``vt0_fps`` along
``heading0_deg``.  The relief is 0.42 % of the weight at the scenario-11
defaults and 1.34 % at the Mach 2 scenario-12 point, and it depends on heading:
an eastbound trim carries less lift than a westbound one.  See
:file:`Aetherion/FlightDynamics/Trim/TrimWeight.h`.

Parameters
~~~~~~~~~~

.. list-table::
   :header-rows: 1
   :widths: 26 10 14 50

   * - Port
     - Unit
     - Default
     - Meaning
   * - ``vt0_fps``
     - ft/s
     - 565.685
     - True airspeed at the trim point.
   * - ``alt0_ft``
     - ft
     - 10013
     - Altitude above MSL at the trim point.
   * - ``lat0_deg``
     - deg
     - 36.0192
     - Initial geodetic latitude (Kitty Hawk, NC).
   * - ``lon0_deg``
     - deg
     - −75.6744
     - Initial geodetic longitude.
   * - ``heading0_deg``
     - deg
     - 45
     - Initial heading, azimuth from North in NED.
   * - ``roll0_deg``
     - deg
     - 0
     - Initial pose roll angle.  Wings-level, as NASA reference simulations 04
       and 05; up to 0.14.1 the default was −0.172°, the initial bank of
       simulation 02.
   * - ``xcg_from_ac_ft``
     - ft
     - 1.132
     - CG aft of the aerodynamic reference centre, (35 % − 25 %) × c̄.  Feeds
       both the trim solve and the aero AC → CG pitch-moment transfer.
   * - ``solver.abs_tol``
     - –
     - 1e-12
     - Radau IIA Newton absolute residual tolerance.
   * - ``solver.rel_tol``
     - –
     - 1e-10
     - Radau IIA Newton relative residual tolerance.
   * - ``solver.max_step_s``
     - s
     - 0
     - Maximum internal sub-step.  0 uses the communication step directly.

Inputs
~~~~~~

.. list-table::
   :header-rows: 1
   :widths: 26 10 64

   * - Port
     - Unit
     - Meaning
   * - ``ctrl.el_deg``
     - deg
     - Elevator deflection.  Seeded to the trim value at initialisation.
   * - ``ctrl.ail_deg``
     - deg
     - Aileron deflection.
   * - ``ctrl.rdr_deg``
     - deg
     - Rudder deflection.
   * - ``ctrl.pwr_pct``
     - %
     - Throttle, 0–100.  Seeded to the trim value at initialisation.

Outputs
~~~~~~~

.. list-table::
   :header-rows: 1
   :widths: 26 10 64

   * - Port
     - Unit
     - Meaning
   * - ``out.alt_m``
     - m
     - Altitude above MSL (WGS-84 geodetic).
   * - ``out.lat_deg``
     - deg
     - Geodetic latitude.
   * - ``out.lon_deg``
     - deg
     - Geodetic longitude.
   * - ``out.yaw_rad``
     - rad
     - ZYX Euler yaw, body → NED.
   * - ``out.pitch_rad``
     - rad
     - ZYX Euler pitch, body → NED.
   * - ``out.roll_rad``
     - rad
     - ZYX Euler roll, body → NED.
   * - ``out.p_rad_s``
     - rad/s
     - Body roll rate with respect to ECI.
   * - ``out.q_rad_s``
     - rad/s
     - Body pitch rate with respect to ECI.
   * - ``out.r_rad_s``
     - rad/s
     - Body yaw rate with respect to ECI.
   * - ``out.v_north_m_s``
     - m/s
     - NED north velocity, Earth-relative.
   * - ``out.v_east_m_s``
     - m/s
     - NED east velocity.
   * - ``out.v_down_m_s``
     - m/s
     - NED down velocity.
   * - ``out.alpha_deg``
     - deg
     - Angle of attack, from atmosphere-relative body velocity.
   * - ``out.beta_deg``
     - deg
     - Sideslip angle.
   * - ``out.mach``
     - –
     - Mach number.
   * - ``out.qbar_Pa``
     - Pa
     - Dynamic pressure.
   * - ``out.vt_m_s``
     - m/s
     - True airspeed.
   * - ``out.aero_Fx_N``
     - N
     - Aerodynamic body-X force.
   * - ``out.aero_Fy_N``
     - N
     - Aerodynamic body-Y force.
   * - ``out.aero_Fz_N``
     - N
     - Aerodynamic body-Z force.
   * - ``out.aero_Mx_Nm``
     - N·m
     - Aerodynamic roll moment.
   * - ``out.aero_My_Nm``
     - N·m
     - Aerodynamic pitch moment, reported **about the AC** (25 % MAC) to match
       the DAVE-ML output convention, not about the CG the EOM uses.
   * - ``out.aero_Mz_Nm``
     - N·m
     - Aerodynamic yaw moment.
   * - ``out.rho_kg_m3``
     - kg/m³
     - Air density, US 1976 standard atmosphere.
   * - ``out.T_K``
     - K
     - Ambient static temperature.
   * - ``out.P_Pa``
     - Pa
     - Ambient static pressure.
   * - ``out.a_m_s``
     - m/s
     - Speed of sound.
   * - ``out.g_m_s2``
     - m/s²
     - Local gravitational acceleration magnitude — **mass attraction only**,
       J2 included, no centrifugal term.  Do not use it to reconstruct specific
       force; see :ref:`fmu_specific_force`.
   * - ``out.specificForce_x_m_s2``
     - m/s²
     - Body-X specific force at the CG.  See :ref:`fmu_specific_force`.
   * - ``out.specificForce_y_m_s2``
     - m/s²
     - Body-Y specific force at the CG.
   * - ``out.specificForce_z_m_s2``
     - m/s²
     - Body-Z specific force at the CG.  Negative in level flight.
   * - ``out.thrust_N``
     - N
     - Net propulsive force along body +X.  The F-16 engine deck carries no
       body-Y or body-Z thrust component, so this is the whole thrust vector.
   * - ``out.mass_kg``
     - kg
     - Vehicle mass.  Constant for this vehicle; published for debugging and
       for symmetry with ``TwoStageRocket``.

.. _fmu_f16autopilot:

F16Autopilot
------------

LQR stability-augmentation inner loop with altitude, airspeed and heading hold,
driving ``F16Plant`` in closed loop.  The control law is purely algebraic, so
the FMU carries no state.

Wiring is direct: every ``fb.*`` input is fed from the identically named
``F16Plant`` output, and the four ``ctrl.*`` outputs go back to the plant's
``ctrl.*`` inputs.

Every input starts at the scenario-11 trim point, so the initial outputs are at
trim when the master sets nothing.  Values written during initialisation mode
are honoured — up to 0.14.1 the FMU re-seeded its inputs on
``fmi2ExitInitializationMode``, which silently replaced, for instance, a
10 000 ft altitude command with 10 013 ft unless the master wrote it again
before the first step.

Control law selection
~~~~~~~~~~~~~~~~~~~~~

The archive carries two NASA control laws.  ``F16_gnc.dml`` is
``F16_control.dml`` with a navigator in front of the heading loop: the course
and lateral-offset commands stop being inputs and are computed from the ownship
position so as to fly a 3 nmi counter-clockwise circle.  The DAVE-ML model has
no "navigator off" state — its switch only picks *which* circle — so the choice
between the two laws is a parameter, fixed at initialisation.

.. list-table::
   :header-rows: 1
   :widths: 24 22 54

   * - ``circumnavigate``
     - Control law
     - Steering
   * - ``false`` (default)
     - ``F16_control.dml``
     - ``cmd.baseChiCmd_deg`` and ``cmd.latOffset_ft``.  Atmos_13.1–13.4.
       ``cmd.circlePoleSW``, ``fb.lat_deg`` and ``fb.lon_deg`` are inert.
   * - ``true``
     - ``F16_gnc.dml``
     - Navigator, from ``fb.lat_deg`` / ``fb.lon_deg``.
       ``cmd.circlePoleSW`` > 0.5 circles the North Pole (Atmos_15); otherwise
       the equator / date-line crossing (Atmos_16).  ``cmd.baseChiCmd_deg`` and
       ``cmd.latOffset_ft`` are ignored.

The closed-loop scenarios 15 and 16 are exercised against the NASA reference
trajectories in :file:`src/FMU/F16Autopilot/test_f16autopilot.py`.

Parameters
~~~~~~~~~~

.. list-table::
   :header-rows: 1
   :widths: 26 10 14 50

   * - Port
     - Unit
     - Default
     - Meaning
   * - ``circumnavigate``
     - Boolean
     - false
     - Load ``F16_gnc.dml`` instead of ``F16_control.dml``.  See above.

Inputs
~~~~~~

.. list-table::
   :header-rows: 1
   :widths: 26 10 14 50

   * - Port
     - Unit
     - Default
     - Meaning
   * - ``cmd.altCmd_ft``
     - ft
     - 10013
     - Commanded altitude.
   * - ``cmd.keasCmd_kt``
     - kt
     - 287.809
     - Commanded equivalent airspeed.
   * - ``cmd.baseChiCmd_deg``
     - deg
     - 45
     - Commanded course angle.  A step on this port is the Atmos_13.3 heading
       change.
   * - ``cmd.latOffset_ft``
     - ft
     - 0
     - Commanded lateral track offset.
   * - ``cmd.circlePoleSW``
     - –
     - 0
     - Navigator select, the DAVE-ML ``circlePoleSW``.  Read only when
       ``circumnavigate`` is true.
   * - ``fb.alt_m``
     - m
     - 3051.96
     - Altitude feedback, from ``F16Plant out.alt_m``.
   * - ``fb.vt_m_s``
     - m/s
     - 172.42
     - True airspeed feedback.
   * - ``fb.rho_kg_m3``
     - kg/m³
     - 0.9042
     - Air density feedback, used for the TAS → EAS conversion.
   * - ``fb.alpha_deg``
     - deg
     - 2.6538
     - Angle-of-attack feedback.
   * - ``fb.beta_deg``
     - deg
     - 0
     - Sideslip feedback.
   * - ``fb.roll_rad``
     - rad
     - 0
     - Roll-attitude feedback.
   * - ``fb.pitch_rad``
     - rad
     - 0.0463
     - Pitch-attitude feedback.
   * - ``fb.yaw_rad``
     - rad
     - 0.7854
     - Yaw-attitude feedback.
   * - ``fb.p_rad_s``
     - rad/s
     - 0
     - Roll-rate feedback.
   * - ``fb.q_rad_s``
     - rad/s
     - 0
     - Pitch-rate feedback.
   * - ``fb.r_rad_s``
     - rad/s
     - 0
     - Yaw-rate feedback.
   * - ``fb.lat_deg``
     - deg
     - 36.0192
     - Geodetic latitude feedback, from ``F16Plant out.lat_deg``.  Read only
       when ``circumnavigate`` is true.
   * - ``fb.lon_deg``
     - deg
     - −75.6744
     - Geodetic longitude feedback, from ``F16Plant out.lon_deg``.  Read only
       when ``circumnavigate`` is true.

Outputs
~~~~~~~

.. list-table::
   :header-rows: 1
   :widths: 26 10 64

   * - Port
     - Unit
     - Meaning
   * - ``ctrl.el_deg``
     - deg
     - Commanded elevator deflection.
   * - ``ctrl.ail_deg``
     - deg
     - Commanded aileron deflection.
   * - ``ctrl.rdr_deg``
     - deg
     - Commanded rudder deflection.
   * - ``ctrl.pwr_pct``
     - %
     - Commanded throttle, 0–100.

.. _fmu_twostagerocket:

TwoStageRocket
--------------

Two-stage rocket ascent with a zero-order-hold per-step update of thrust,
propellant flow and spatial inertia, plus a discontinuous mass drop at stage
separation.  There are no control inputs: the vehicle flies an open-loop gravity
turn from the initial pitch attitude.

Parameters
~~~~~~~~~~

.. list-table::
   :header-rows: 1
   :widths: 26 10 14 50

   * - Port
     - Unit
     - Default
     - Meaning
   * - ``lat0_deg``
     - deg
     - 0
     - Launch geodetic latitude.
   * - ``lon0_deg``
     - deg
     - 0
     - Launch geodetic longitude.
   * - ``alt0_m``
     - m
     - 0
     - Launch altitude above MSL.
   * - ``azimuth0_deg``
     - deg
     - 90
     - Initial heading, azimuth from North.
   * - ``pitch0_deg``
     - deg
     - 55.22
     - Initial pitch, nose-up from horizontal.
   * - ``roll0_deg``
     - deg
     - 0
     - Initial roll angle.
   * - ``vNorth0_mps``
     - m/s
     - 0
     - Initial NED north velocity.
   * - ``vEast0_mps``
     - m/s
     - 0
     - Initial NED east velocity.
   * - ``vDown0_mps``
     - m/s
     - 0
     - Initial NED down velocity.
   * - ``solver.max_step_s``
     - s
     - 0
     - Maximum internal sub-step.  0 uses the communication step directly.
   * - ``stg2.ignition_time_s``
     - s
     - 0
     - **Tunable.**  Absolute simulation time gating stage-2 ignition; 0 ignites
       immediately once stage 1 has separated.  Set to
       (end time − S2 burn duration) to reproduce the NASA TM coast-then-fire
       sequencing.

Outputs
~~~~~~~

Ports shared with ``F16Plant`` — ``out.alt_m``, ``out.lat_deg``, ``out.lon_deg``,
``out.g_m_s2``, the three Euler angles, the three body rates, the three NED
velocities, ``out.a_m_s``, ``out.rho_kg_m3``, ``out.P_Pa``, ``out.T_K``, the six
``out.aero_*`` forces and moments, ``out.mach``, ``out.qbar_Pa``, ``out.vt_m_s``
and the three ``out.specificForce_*`` channels — carry the same meaning and units as in
:ref:`fmu_f16plant`, with two differences: ``out.aero_My_Nm`` is reported about
the MRC rather than an aerodynamic centre, and this vehicle's mass genuinely
varies.  The ports below are specific to this model.

.. list-table::
   :header-rows: 1
   :widths: 26 10 64

   * - Port
     - Unit
     - Meaning
   * - ``out.altRate_m_s``
     - m/s
     - Altitude rate, :math:`\dot{h} = -v_\mathrm{down}`.
   * - ``out.thrust_N``
     - N
     - Total axial thrust along body +x, zero-order held over the step just
       taken.
   * - ``out.mdot_kgs``
     - kg/s
     - Propellant consumption rate, positive while burning.
   * - ``out.mass_kg``
     - kg
     - Vehicle mass.  Drops by 35 000 kg at stage separation.
   * - ``out.stg1_fuel_used_kg``
     - kg
     - Cumulative stage-1 propellant consumed.
   * - ``out.stg2_fuel_used_kg``
     - kg
     - Cumulative stage-2 propellant consumed.
   * - ``out.staged``
     - Boolean
     - True once the stage-1 dry mass has been jettisoned.  Discrete.

.. note::

   On the single step in which separation occurs, ``out.thrust_N`` reports the
   zero-order-hold thrust that was actually applied over that step — stage 1 —
   while ``out.mass_kg`` has already dropped to the post-separation value.
   ``out.specificForce_*`` deliberately does not pair those two: it uses the
   post-separation thrust, which is what matches the post-separation mass, so it
   carries no spurious stage-1-thrust-over-stage-2-mass spike.  Away from that
   one step the two agree exactly.
