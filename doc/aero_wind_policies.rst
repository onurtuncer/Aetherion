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

PowerLawWindShear
~~~~~~~~~~~~~~~~~

Wind magnitude varies with geocentric altitude following a power law:

.. math::

   \mathbf{v}_\mathrm{wind,ECEF}(\mathbf{r},t)
   = \mathbf{v}_\mathrm{ref,ECEF}
     \left(\frac{h}{h_\mathrm{ref}}\right)^n,

where :math:`h = \|\mathbf{r}_\mathrm{ECI}\| - R_e` is the geocentric
altitude, :math:`\mathbf{v}_\mathrm{ref,ECEF}` is the wind vector at the
reference altitude :math:`h_\mathrm{ref}`, and :math:`n` is the shear
exponent.

For NASA TM-2015-218675 Scenario 8 (70 ft/s at 30 000 ft), a least-squares
fit to the reference data gives :math:`n \approx 4/3 \approx 1.333`.

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
       WindAwareDragPolicy<PowerLawWindShear>;

   ShearPolicy p{ CD, S_ref,
                  PowerLawWindShear::from_ned(
                      0.0, 21.336, 0.0,   // NED wind at h_ref [m/s]
                      lat0, lon0,
                      9144.0,             // h_ref [m]
                      4.0/3.0) };         // shear exponent


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
   * - ``WindAwareDragPolicy<PowerLawWindShear>``
     - ✓
     - ✗
     - Earth rotation + shear wind
     - 8
   * - ``WindAwareDragPolicy<GeodesicCallbackWind>``
     - ✓
     - ✗
     - Earth rotation + API wind
     - user-defined
