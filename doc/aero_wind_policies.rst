.. ------------------------------------------------------------------------------
.. Project: Aetherion
.. Copyright (c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
..
.. SPDX-License-Identifier: MIT
.. License-Filename: LICENSE
.. ------------------------------------------------------------------------------

.. _aero_wind_policies:

Aerodynamic and Wind Policies
==============================

All external forces and moments on the rigid body are expressed as
**spatial wrenches** in the body frame:

.. math::

   \mathbf{f}_B
   = \begin{bmatrix} \boldsymbol{\tau}_B \\ \mathbf{F}_B \end{bmatrix}
   \in \mathbb{R}^6,

where :math:`\boldsymbol{\tau}_B \in \mathbb{R}^3` are moments (indices 0–2)
and :math:`\mathbf{F}_B \in \mathbb{R}^3` are forces (indices 3–5),
consistent with the Featherstone spatial-vector convention used throughout
Aetherion.

Aerodynamic wrenches are computed by **AeroPolicy** objects passed as
template parameters to :cpp:class:`RigidBody::VectorField`.
Wind effects are factored out into a separate **WindModel** layer so that
any combination of drag model and wind profile can be composed without
modifying the core integrator.


Atmosphere-Relative Velocity
-----------------------------

The key quantity for all aerodynamic force computations is the
**atmosphere-relative velocity** in the body frame.  The atmosphere
co-rotates with Earth; a body orbiting without atmosphere-relative motion
experiences zero drag:

.. math::

   \mathbf{v}_\mathrm{rel}
   = \underbrace{\mathbf{v}_B}_{\text{ECI velocity, body frame}}
   - \underbrace{R^T\!\bigl(\boldsymbol{\omega}_\oplus \times
                             \mathbf{r}_\mathrm{ECI}\bigr)}_{\text{Earth-surface velocity}}
   - \underbrace{R^T R_\mathrm{ECEF\to ECI}(t)\,
                 \mathbf{v}_\mathrm{wind,ECEF}}_{\text{ambient wind}},

where

* :math:`\mathbf{v}_B = \nu_B^{3:5}` is the body-frame linear velocity (lower
  half of the body twist),
* :math:`R \in SO(3)` is the rotation from body to ECI frame,
* :math:`\boldsymbol{\omega}_\oplus = [0,\,0,\,\omega_\oplus]^\top` is Earth's
  spin vector in the ECI frame (:math:`\omega_\oplus = 7.2921150 \times 10^{-5}` rad/s),
* :math:`\mathbf{r}_\mathrm{ECI}` is the vehicle's ECI position, and
* :math:`\mathbf{v}_\mathrm{wind,ECEF}` is the ambient wind in the ECEF frame,
  rotated to ECI at the current Earth Rotation Angle
  :math:`\theta_\mathrm{ERA}(t) = \omega_\oplus\,t`.

When there is no wind, :math:`\mathbf{v}_\mathrm{wind,ECEF} = \mathbf{0}` and
the expression reduces to the **Earth-surface velocity subtraction** used in
:cpp:class:`DragOnlyAeroPolicy`.

The **true airspeed** (atmosphere-relative speed) reported in the output CSV
is

.. math::

   V \;=\; \|\mathbf{v}_\mathrm{rel}\|.

All drag and damping formulas below use :math:`\mathbf{v}_\mathrm{rel}` and
:math:`V`.


.. _sec_drag_only:

Drag-Only Policy (``DragOnlyAeroPolicy``)
------------------------------------------

Models a sphere or any body with purely resistive (no-lift) aerodynamics.
The drag force in the body frame is

.. math::

   \mathbf{F}_\mathrm{drag}
   = -\tfrac{1}{2}\,\rho(h)\,C_D\,S_\mathrm{ref}\;
     V\;\mathbf{v}_\mathrm{rel},

where :math:`\rho(h)` is the US Standard Atmosphere 1976 density at the
geocentric altitude :math:`h \approx \|\mathbf{r}_\mathrm{ECI}\| - R_e`.

The corresponding body-frame wrench has zero moment components:

.. math::

   \mathbf{f}_\mathrm{aero}
   = \begin{bmatrix} \mathbf{0}_3 \\ \mathbf{F}_\mathrm{drag} \end{bmatrix}.

**Parameters**: :math:`C_D` (drag coefficient), :math:`S_\mathrm{ref}`
(reference area [m²]).

**AD-safe**: yes — the square-root singularity at :math:`V = 0` is avoided by
adding a small floor :math:`\varepsilon = 10^{-15}` before taking
:math:`\sqrt{V^2 + \varepsilon}`.

**Used in**: Scenario 6 (sphere with drag), Scenarios 9 & 10 (cannonball).


.. _sec_brick_damping:

Drag and Rotary-Damping Policy (``BrickDampingAeroPolicy``)
------------------------------------------------------------

Extends :ref:`sec_drag_only` with aerodynamic moment damping proportional
to the body angular rates.  This is the standard non-dimensional rate
formulation used for the tumbling-brick scenarios.

Damping moments use the **standard dimensionless rate convention**:

.. math::

   \hat{p} = \frac{p\,b}{2V}, \quad
   \hat{q} = \frac{q\,\bar{c}}{2V}, \quad
   \hat{r} = \frac{r\,b}{2V},

where :math:`b` is the span reference length and :math:`\bar{c}` is the
chord reference length.

The rolling, pitching, and yawing damping moments are

.. math::

   L = \tfrac{1}{2}\rho\,V^2\,S\,b\;C_{l_p}\hat{p}
     = \tfrac{1}{4}\rho\,V\,S\,b^2\;C_{l_p}\;p,

.. math::

   M = \tfrac{1}{2}\rho\,V^2\,S\,\bar{c}\;C_{m_q}\hat{q}
     = \tfrac{1}{4}\rho\,V\,S\,\bar{c}^2\;C_{m_q}\;q,

.. math::

   N = \tfrac{1}{2}\rho\,V^2\,S\,b\;C_{n_r}\hat{r}
     = \tfrac{1}{4}\rho\,V\,S\,b^2\;C_{n_r}\;r.

The full wrench combines drag force and damping moments:

.. math::

   \mathbf{f}_\mathrm{aero}
   = \begin{bmatrix} L \\ M \\ N \\ F_{\mathrm{drag},x} \\
                     F_{\mathrm{drag},y} \\ F_{\mathrm{drag},z} \end{bmatrix}.

**Parameters**: :math:`C_D,\,S` (drag); :math:`b,\,\bar{c}` (reference
lengths); :math:`C_{l_p},\,C_{m_q},\,C_{n_r}` (rotary derivatives).

**Note**: Cross-coupling derivatives (:math:`C_{l_r},\,C_{n_p}`, etc.) are
stored in :cpp:struct:`AerodynamicParameters` and default to zero.

**Used in**: Scenario 3 (tumbling brick with damping).


.. _sec_wind_models:

Wind Models
-----------

All wind-aware policies receive the ambient wind via a **WindModel** —
a lightweight struct providing a single templated method:

.. code-block:: cpp

   template<class S>
   Eigen::Matrix<S,3,1> velocity_ecef(
       const Eigen::Matrix<S,3,1>& r_eci,   // ECI position [m]
       S t_s) const;                          // simulation time [s]

The method returns the wind velocity in the **ECEF frame** [m/s] and must
be callable for both ``S = double`` (evaluation) and ``S = CppAD::AD<double>``
(Jacobian recording).

Three built-in models are provided.

ZeroWind
~~~~~~~~

.. math::

   \mathbf{v}_\mathrm{wind,ECEF}(\mathbf{r},t) = \mathbf{0}.

Calm atmosphere.  Reduces :ref:`sec_wind_aware` to :ref:`sec_drag_only`.

ConstantECEFWind
~~~~~~~~~~~~~~~~

.. math::

   \mathbf{v}_\mathrm{wind,ECEF}(\mathbf{r},t) = \mathbf{v}_0
   \quad \text{(constant ECEF vector)}.

The helper :cpp:func:`ConstantECEFWind::from_ned` converts a NED wind
specification at the launch geodetic position into the ECEF frame:

.. math::

   \mathbf{v}_0
   = R_\mathrm{NED\to ECEF}(\phi_0,\lambda_0)\,
     \begin{bmatrix} v_N \\ v_E \\ v_D \end{bmatrix},

where :math:`\phi_0,\,\lambda_0` are the launch geodetic latitude and
longitude.

**Used in**: Scenario 7 (steady 20 ft/s eastward wind).

LinearWindShear
~~~~~~~~~~~~~~~

Wind varies **linearly** with geocentric altitude in each NED component:

.. math::

   v_N(h) = \alpha_N \, h + \beta_N, \quad
   v_E(h) = \alpha_E \, h + \beta_E,

where :math:`h = \|\mathbf{r}_\mathrm{ECI}\| - R_e` is the geocentric
altitude, :math:`\alpha` is the altitude gradient [m/s per m], and
:math:`\beta` is the sea-level intercept [m/s].  The NED wind is converted
to ECEF once at construction from the launch latitude/longitude.

For NASA TM-2015-218675 Scenario 8 the formula is given explicitly:

   *"Vwind = (0.003h − 20) ft/s from west; h is height MSL in ft."*

In SI units: :math:`v_E(h) = 0.003\,h_\text{m} - 6.096` m/s
(:math:`\alpha_E = 0.003`, :math:`\beta_E = -6.096`, :math:`\alpha_N = \beta_N = 0`).
The wind is eastward above 2 032 m (6 667 ft), zero there, and westward below.

**Used in**: Scenario 8 (2D wind shear).

GeodesicCallbackWind
~~~~~~~~~~~~~~~~~~~~~

Wraps an arbitrary user-provided callable

.. code-block:: cpp

   std::function<Eigen::Vector3d(double lat_rad,
                                 double lon_rad,
                                 double alt_m,
                                 double t_s)>

that returns the NED wind at the current geodetic position.  This enables
integration with **external weather services** or pre-computed atmospheric
data fields.

**AD compatibility strategy** — the implicit Newton solver in the Radau IIA
integrator records a CppAD tape that calls the wind model with
:math:`S = \mathtt{AD<double>}` values.  Since an external API cannot be
called with ``AD`` arguments, the implementation uses the
*frozen-Jacobian approximation*:

1. During the *double* path (residual evaluation), the callback is invoked
   with plain ``double`` coordinates and the resulting ECEF wind vector is
   cached.
2. During the *AD* path (Jacobian recording), the cached value is returned
   as a constant.

This is equivalent to treating the wind as piecewise-constant over each
Newton step — a standard and physically reasonable approximation since wind
fields change on timescales much longer than an integration step.

To add a custom wind profile, implement ``velocity_ecef(r_eci, t)`` and
register the type:

.. code-block:: cpp

   // In namespace Aetherion::FlightDynamics:
   template<> struct is_wind_model<MyWind> : std::true_type {};


.. _sec_wind_aware:

Wind-Aware Drag Policy (``WindAwareDragPolicy<Wind>``)
-------------------------------------------------------

General drag policy that accepts any registered ``WindModel`` as a template
parameter.  It subsumes :ref:`sec_drag_only` (using ``ZeroWind``) and
extends it to steady or shear winds.

The atmosphere-relative velocity is computed as

.. math::

   \mathbf{v}_\mathrm{rel}
   = \mathbf{v}_B
   - R^T(\boldsymbol{\omega}_\oplus \times \mathbf{r}_\mathrm{ECI})
   - R^T R_z(\theta_\mathrm{ERA})\,
     \mathbf{v}_\mathrm{wind,ECEF}(\mathbf{r}_\mathrm{ECI},\,t),

where :math:`R_z(\theta) = R_\mathrm{ECEF\to ECI}(\theta)` rotates the ECEF
wind into the ECI frame at the current ERA
:math:`\theta_\mathrm{ERA}(t) = \omega_\oplus\,t`.

The drag force is then identical to :ref:`sec_drag_only`:

.. math::

   \mathbf{F}_\mathrm{drag}
   = -\tfrac{1}{2}\,\rho(h)\,C_D\,S_\mathrm{ref}\;
     \|\mathbf{v}_\mathrm{rel}\|\;\mathbf{v}_\mathrm{rel}.

The policy is instantiated as, for example:

.. code-block:: cpp

   using ShearPolicy =
       WindAwareDragPolicy<LinearWindShear>;

   ShearPolicy p{ CD, S_ref,
                  LinearWindShear::from_ned(
                      0.0,    0.003,    // gradient  (N, E) [m/s per m]
                      0.0,   -6.096,   // intercept (N, E) [m/s at h=0]
                      lat0, lon0) };


.. _vehicle_environment:

Environment on the Vehicle Policies
------------------------------------

The sphere policies above take their wind through a ``WindModel`` template
parameter.  The two vehicle policies, :cpp:class:`FlightDynamics::F16AeroPolicy`
and :cpp:class:`Examples::TwoStageRocket::RocketAeroPolicy`, are concrete
types held by the example and FMU vector fields, so they carry their
environment as **runtime state set between integration steps**, in the same
way the control-surface deflections are:

.. code-block:: cpp

   policy.setWindECEF(v_wind_ecef);            // steady wind, ECEF [m/s]
   policy.setGust(gust);                       // Environment::GustState, body axes (F-16 only)
   policy.setAtmosphereOffsets({dT_K, dP_Pa}); // Environment::AtmosphereOffsets

All three default to calm air on a standard day, and the calm, standard-day
evaluation is **bit-identical** to a policy without these members: the wind
and gust subtractions are skipped when the vectors are zero, and the zero
offset selects the published US1976 layer table unchanged.  The NASA
check-case results are therefore unaffected by the feature.

Air-relative velocity and rates
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The policy forms

.. math::

   \mathbf{v}_\mathrm{rel}
   = \mathbf{v}_B
   - R^T\!\bigl(\boldsymbol{\omega}_\oplus \times \mathbf{r}_\mathrm{ECI}\bigr)
   - R^T R_\mathrm{ECEF\to ECI}(t)\,\mathbf{v}_\mathrm{wind,ECEF}
   - \mathbf{v}_\mathrm{gust,B},
   \qquad
   \boldsymbol{\omega}_\mathrm{air}
   = \boldsymbol{\omega}_B - R^T\boldsymbol{\omega}_\oplus
   - \boldsymbol{\omega}_\mathrm{gust,B},

and exposes both through ``airRelativeVelocity_B(g, nu_B, t)`` and
``airRelativeRates_B(g, nu_B)``, so that whoever reports :math:`\alpha`,
:math:`\beta` and TAS (the FMU output block, :cpp:func:`MakeSnapshot1`) uses
the vector the forces were computed from.  Before 0.16.0 the F-16 FMU kept
its own copy of the formula; it now calls the policy.

The same two functions live in ``FlightDynamics/Policies/AirRelative.h`` and
are shared with :cpp:class:`F16PropPolicy`, which forms the engine's Mach
number from the same air-relative velocity and the same (offset) speed of
sound.  This matters: an engine that formed Mach from the ground-relative
velocity would deliver a different thrust in a tailwind than the trim
assumed, and the aircraft would drift out of trim by tens of metres a minute.
The propulsion policy therefore carries the same ``setWindECEF`` /
``setGust`` / ``setAtmosphereOffsets`` and the FMU sets both policies alike.

A steady wind is given as an ECEF vector.  The FMUs take it as NED components
at the initial position and convert once with
:cpp:func:`ConstantECEFWind::from_ned`, which freezes the local frame at the
start point (over a 200 s F-16 run the position moves about 60 km, a few
hundredths of a degree in "north").  The trim is air-relative and unchanged
by wind; the initial ground velocity is the trim airspeed along the heading
plus the wind, so the aircraft crabs in a crosswind and its ground speed
drops in a headwind while :math:`\alpha`, :math:`V_\mathrm{TAS}` and the
control settings stay at their calm values.

Dryden turbulence
~~~~~~~~~~~~~~~~~

:cpp:class:`Environment::DrydenTurbulence` produces a
:cpp:struct:`Environment::GustState`: three linear gust velocities
:math:`(u_g, v_g, w_g)` in body axes and the angular velocity of the air
:math:`(p_\mathrm{air}, q_\mathrm{air}, r_\mathrm{air})`.  The spectra are
the MIL-F-8785C forms, one-sided in spatial frequency :math:`\Omega`
[rad/m]:

.. math::

   \Phi_u(\Omega) = \sigma_u^2\,\frac{2L_u}{\pi}\,\frac{1}{1+(L_u\Omega)^2},
   \qquad
   \Phi_{v,w}(\Omega) = \sigma^2\,\frac{L}{\pi}\,
   \frac{1+3(L\Omega)^2}{\bigl(1+(L\Omega)^2\bigr)^2},

each integrating to :math:`\sigma^2`, so :math:`\sigma` is the RMS gust.
Under Taylor's frozen-field hypothesis :math:`\omega = V\Omega`, and every
shaping filter has a corner :math:`V/L` that is re-evaluated at each step from
the current airspeed.  The rotational components follow the specification:
:math:`p_\mathrm{air}` is a first-order process on :math:`\Phi_p` with corner
:math:`\pi V/4b`; :math:`q_\mathrm{air}` and :math:`r_\mathrm{air}` are the
lagged streamwise gradients of :math:`w_g` and :math:`v_g` (corners
:math:`\pi V/4b` and :math:`\pi V/3b`, :math:`b` the wingspan).

.. note::

   MIL-HDBK-1797 writes the :math:`v` and :math:`w` spectra with
   :math:`2L_v`, :math:`2L_w` and quotes :math:`2L_w = h` at low altitude.
   Aetherion uses the MIL-F-8785C definitions throughout: above 2000 ft,
   :math:`L_u = L_v = L_w = 1750` ft (533.4 m), which is the parameter default.

**Sign convention.**  The three angular components are the angular velocity
of the air, and the aero model sees
:math:`\boldsymbol{\omega}_B - \boldsymbol{\omega}_\mathrm{gust}`.  With
:math:`p_\mathrm{air} = \partial w_g/\partial y`, a downward gust growing
toward the right wing gives the aircraft a negative roll rate relative to the
air, which roll damping turns into a right-rolling moment, as the lift
asymmetry would; the same holds for the pitch and yaw components.  The
specification leaves the sign of :math:`q_g` and :math:`r_g` to the reader;
this is the one consistent with the frozen field, and it is pinned by a test
against the F-16 model (``test_F16Environment.cpp``).

**Discretisation.**  The filters are Euclidean, stochastic states; they do not
belong inside the Lie-group integrator.  They are stepped **between
integrator sub-steps** by the exact zero-order-hold sampling of the linear
stochastic filters (Van Loan's method for the transition matrix and the
process-noise covariance), and the gust is held constant over the sub-step,
where it is a constant on the AD tape.  The sample statistics are therefore
independent of :math:`\Delta t` (a test checks 0.01 s against 0.04 s), but
the step must be **fixed** from call to call: a varying step changes the
process, not just its realisation.  The F-16 FMU warns once if the
communication step varies while turbulence is on.  Nothing is claimed about
the integrator's order under turbulence; there is none to preserve for
stochastic forcing.

**Parameters** are the six spectral numbers (three :math:`\sigma`, three
:math:`L`), the wingspan and a seed.  :cpp:func:`DrydenLowAltitudeParameters`
implements the specification's low-altitude rules
(:math:`\sigma_w = 0.1\,W_{20}`, :math:`L_w = h`, the
:math:`(0.177 + 0.000823\,h)` factors).  Above 2000 ft the intensities come
from a probability-of-exceedance chart (MIL-F-8785C Figure 7) that is not
transcribed here; set the six numbers directly for that regime.  The noise
stream is ``std::mt19937_64`` through a Box-Muller transform written out in
the header, so a seed reproduces the same record on every platform.

**Tests** (``tests/Environment/test_DrydenTurbulence.cpp``): the sample
variance of each linear channel equals :math:`\sigma^2` and that of
:math:`p_\mathrm{air}` equals :math:`\Phi_p` integrated over frequency, to
4 %; an averaged periodogram matches :math:`\Phi_u` and :math:`\Phi_w` at
four frequencies spanning the corner to 15 %, including the
:math:`(1+3x^2)` shape of the :math:`w` channel; the statistics agree between
step sizes; the same seed gives the same record; the filter states survive
``states()``/``setStates()``.

Non-standard atmosphere
~~~~~~~~~~~~~~~~~~~~~~~~

:cpp:func:`Environment::US1976Atmosphere` has a two-argument overload taking
:cpp:struct:`Environment::AtmosphereOffsets`: an ISA temperature deviation
``deltaT_K`` added uniformly to every layer base temperature (lapse rates
unchanged), and a sea-level pressure offset ``deltaP_sl_Pa`` (QNH minus
1013.25 hPa).  The pressure is **re-integrated hydrostatically**, not scaled:
with the temperature profile shifted, every scale height changes, so the
layer base pressures are recomputed by walking the layers from the shifted
sea-level pressure with the same closed forms the layer evaluation uses.  To
stay continuous with the published (rounded) base-pressure table, each base
is the published value scaled by the ratio of the re-integrated base with
offsets to the re-integrated base without, which reduces to the table
exactly when the offsets vanish.  Density and speed of sound follow from the
gas law and the shifted temperature.

Two numbers to hold on to: ISA + 15 K at sea level gives
:math:`\rho = 1.1644` kg/m³, and a 10 hPa low (``deltaP_sl_Pa = -1000``)
makes a barometer that inverts the standard ISA read about **84 m** at the
surface, which is the pressure-altitude error a filter has to fuse away.  The
tests (``tests/Environment/test_atmosphere_offsets.cpp``) pin these, the
bit-identity of the zero-offset day, :math:`\mathrm{d}p/\mathrm{d}h = -\rho
g_0` inside every layer for several offset pairs, and continuity across
layer bases.

The offsets are held by the aero policy (``setAtmosphereOffsets``) and by
:cpp:class:`TrimSolver` (``setAtmosphereOffsets``), which must agree, and
both snapshots read them from the policy so the reported ``P_Pa``, ``T_K``,
``rho_kg_m3`` and ``a_m_s`` are the ones the forces used.  On a hot day the
F-16 trims at a higher :math:`\alpha` and throttle.  One documented
inconsistency remains: the F-16 propulsion table is indexed on geometric
altitude and Mach for a standard day, so on a hot day the engine makes the
standard-day thrust at that altitude.


Snapshot Air-Data Corrections
------------------------------

:cpp:func:`MakeSnapshot1` computes the reported **true airspeed** (TAS),
Mach number, and dynamic pressure from the Earth-relative NED velocity
:math:`\mathbf{v}_\mathrm{NED}`:

.. math::

   V_\mathrm{TAS} = \|\mathbf{v}_\mathrm{NED}\|
   \quad\text{(no-wind cases)}.

For wind-aware simulators (Scenarios 7 and 8), the snapshot
:cpp:func:`snapshot()` override subtracts the ambient wind before computing
air-data quantities:

.. math::

   V_\mathrm{TAS} = \|\mathbf{v}_\mathrm{NED} - \mathbf{v}_\mathrm{wind,NED}(h)\|,

so that the reported TAS, Mach, and :math:`\bar{q}` match the NASA reference
convention.  For Scenario 8 (power-law shear), the wind-at-altitude correction
uses the same exponent :math:`n` as the policy.

For the vehicle policies (:ref:`vehicle_environment`), the two-policy
overloads of :cpp:func:`MakeSnapshot1` and :cpp:func:`MakeSnapshot2` read the
steady wind, the gust and the atmosphere offsets from the aero policy itself,
subtract the wind (rotated ECEF → NED) and the gust (rotated body → NED) from
the NED velocity before forming TAS, Mach and :math:`\bar{q}`, and evaluate
the atmosphere with the policy's offsets.  With the default environment every
number is unchanged bit for bit.


Policy Composition Summary
---------------------------

.. list-table::
   :header-rows: 1
   :widths: 28 18 18 18 18

   * - Policy
     - Drag force
     - Damping moments
     - Wind subtraction
     - Scenario(s)
   * - ``ZeroAeroPolicy``
     - ✗
     - ✗
     - ✗
     - 1, 2, 9, 10 (dragless)
   * - ``DragOnlyAeroPolicy``
     - ✓
     - ✗
     - Earth rotation only
     - 6
   * - ``BrickDampingAeroPolicy``
     - ✓ (opt.)
     - ✓
     - Earth rotation only
     - 3
   * - ``WindAwareDragPolicy<ZeroWind>``
     - ✓
     - ✗
     - Earth rotation only
     - —
   * - ``WindAwareDragPolicy<ConstantECEFWind>``
     - ✓
     - ✗
     - Earth rotation + constant wind
     - 7
   * - ``WindAwareDragPolicy<LinearWindShear>``
     - ✓
     - ✗
     - Earth rotation + linear altitude shear
     - 8
   * - ``WindAwareDragPolicy<GeodesicCallbackWind>``
     - ✓
     - ✗
     - Earth rotation + API wind
     - user-defined
   * - ``F16AeroPolicy``
     - ✓ (DAVE-ML)
     - ✓
     - Earth rotation + steady wind + Dryden gust; ISA offsets
     - 11–16, F16Plant FMU
   * - ``RocketAeroPolicy``
     - ✓ (DAVE-ML)
     - ✗
     - Earth rotation + steady wind; ISA offsets
     - 17, TwoStageRocket FMU
