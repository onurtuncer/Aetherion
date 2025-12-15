.. _unified-notation:

Unified Notation
================

:Author: Onur Tuncer, PhD
:Date: December 15\ :sup:`th`, 2025

.. contents::
   :local:
   :depth: 2

Goal
----

A single consistent notation that:

- uses *spatial algebra* (Featherstone notation) for rigid-body dynamics,
- treats configuration variables as living on a *(product) manifold*,
- supports *Lie-group-aware* integration (RKMK) without ad-hoc constraint fixes
  (e.g. re-orthonormalization).

Unification is achieved by interpreting the Featherstone 6D spatial vectors as
*coordinates of the Lie algebra* :math:`\Lie{se}(3)` (and its dual
:math:`\Lie{se}(3)^{\*}`), and poses as elements of :math:`\SE(3)`.

State as a product manifold
---------------------------

For a single rigid body, choose the configuration

.. math::

   g \in \SE(3), \qquad
   g =
   \begin{bmatrix}
   R & p\\
   0 & 1
   \end{bmatrix}_{4x4},
   \quad R\in \SO(3),\ p\in\R^3.

A typical flight / GNC state includes additional Euclidean components (velocities,
biases, masses, aerodynamic states, etc.):

.. math::

   x = (g,\ y) \in \SE(3) \times \R^m.

For multi-body systems (or multiple frames), configuration becomes a product
manifold:

.. math::

   x \in \underbrace{\SE(3)\times\cdots\times\SE(3)}_{N\ \text{bodies / frames}} \times \R^m.

This is a *product manifold* viewpoint: Lie-group factors for configuration,
Euclidean factors for everything else.

Spatial motion vectors as coordinates of :math:`\Lie{se}(3)`
------------------------------------------------------------

A Featherstone spatial motion vector (twist) is

.. math::

   \twist =
   \begin{bmatrix}
   \omega\\
   v
   \end{bmatrix}\in\R^6,
   \quad
   \omega\in\R^3,\ v\in\R^3.

We identify this with an element of the Lie algebra :math:`\Lie{se}(3)` via the
*hat* map:

.. math::

   (\cdot)^\wedge:\R^6\to \Lie{se}(3),\qquad
   \twist^\wedge \;=\;
   \begin{bmatrix}
   [\omega]_\times & v\\
   0 & 0
   \end{bmatrix}_{4x4}.

In contrast, :math:`(\cdot)^\vee` maps :math:`\Lie{se}(3)` back to :math:`\R^6`.

**Key unification point:**

Spatial *motion* vectors are simply coordinates for :math:`\Lie{se}(3)`.
Spatial *forces* (wrenches) are the coordinates for the dual space
:math:`\Lie{se}(3)^{\*}`:

.. math::

   \wrench =
   \begin{bmatrix}
   n\\
   f
   \end{bmatrix}\in\R^6
   \quad\leftrightarrow\quad
   \Lie{se}(3)^{\*}.

Kinematics on :math:`\SE(3)` using spatial velocity
---------------------------------------------------

Write kinematics in *left-trivialized* (body) form:

.. math::

   \dot g = g\,\twist^\wedge.

This equation is the clean bridge between Lie theory and Featherstone notation:

- :math:`g\in\SE(3)` is the pose (position + orientation),
- :math:`\twist\in\R^6` is the spatial twist (body spatial velocity),
- :math:`\twist^\wedge\in\Lie{se}(3)` is the corresponding Lie algebra element.

Featherstone cross operators as :math:`\ad` and :math:`\ad^{\*}`
----------------------------------------------------------------

Featherstone defines bi-linear operators:

- motion cross product: :math:`\twist \times (\cdot)` mapping :math:`\R^6\to \R^6`,
- force cross product: :math:`\twist \times^{\*} (\cdot)` mapping :math:`\R^6\to \R^6`.

In Lie notation these are precisely the adjoint actions:

.. math::

   \ad_{\twist}(\cdot)\ \equiv\ \twist \times (\cdot),
   \qquad
   \ad_{\twist}^{\*}(\cdot)\ \equiv\ \twist \times^{\*}(\cdot).

Therefore “spatial algebra dynamics” can be re-written in a Lie-algebraic form
without changing the underlying computations.

Rigid-body dynamics in unified form
-----------------------------------

Let :math:`I\in\R^{6\times 6}` be the spatial inertia, :math:`\twist` the body
twist, and :math:`\wrench` the net wrench. The standard Featherstone equation

.. math::

   I\,\dot{\twist} + \twist\times^{\*}(I\twist) = \wrench

is equivalently

.. math::

   I\,\dot{\twist} + \ad_{\twist}^{\*}\!\big(I\twist\big) = \wrench.

This is the central “unified” dynamics statement:

- pose lives on :math:`\SE(3)`,
- velocities live in :math:`\Lie{se}(3)\cong\R^6`,
- forces live in :math:`\Lie{se}(3)^{\*}\cong\R^6`,
- coupling terms use :math:`\ad^{\*}`, matching Featherstone's cross operator.

ODE on the product manifold :math:`\SE(3)\times\R^m`
----------------------------------------------------

A general coupled system can be written as

.. math::

   \dot g = g\,\Omega(g,y)^\wedge,\qquad
   \dot y = f(g,y),

where :math:`\Omega(g,y)\in\R^6` is a spatial twist (often produced by vehicle
dynamics + constraints + controls) and :math:`f(g,y)\in\R^m` is the Euclidean part
(biases, masses, actuator states, aero coefficients, etc.).

RKMK integration in the same notation
-------------------------------------

Runge–Kutta–Munthe-Kaas (RKMK) integrates the Lie-group factor by evolving an
algebra increment

.. math::

   \eta \in \Lie{se}(3)\quad\text{(or its coordinate vector in }\R^6\text{)}

and updating the pose using the group exponential:

.. math::

   g_{n+1} = g_n\,\Exp(\eta_{n+1}).

The Euclidean part advances by standard Runge–Kutta addition:

.. math::

   y_{n+1} = y_n + \Delta y.

Stage reconstruction on :math:`\SE(3)\times\R^m`
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Let :math:`(a_{ij},b_i,c_i)` be a Runge–Kutta tableau with :math:`s` stages and
step size :math:`h`. Maintain stage variables

.. math::

   \eta_i \in \Lie{se}(3),\qquad \rho_i\in\R^m,

and define stage states

.. math::

   g_i = g_n\,\Exp(\eta_i),\qquad y_i = y_n + \rho_i.

Stage rates and :math:`\dexp^{-1}`
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

RKMK evaluates the vector field in the algebra (a vector space) via the inverse
differential of the exponential:

.. math::

   K_i = \dexp_{\eta_i}^{-1}\!\big(\Omega(g_i,y_i)\big)\in \Lie{se}(3),
   \qquad
   k_i = f(g_i,y_i)\in\R^m.

(If one stores algebra elements as 6-vectors, :math:`K_i` and :math:`\Omega` can
be interpreted as :math:`\R^6` coordinates.)

Explicit stage accumulation
^^^^^^^^^^^^^^^^^^^^^^^^^^^

For an explicit Runge–Kutta tableau:

.. math::

   \eta_i = h\sum_{j=1}^{i-1} a_{ij}K_j,\qquad
   \rho_i = h\sum_{j=1}^{i-1} a_{ij}k_j.

Then the final update is

.. math::

   \eta_{n+1} = h\sum_{i=1}^{s} b_i K_i,\qquad
   y_{n+1} = y_n + h\sum_{i=1}^{s} b_i k_i,\qquad
   g_{n+1} = g_n\,\Exp(\eta_{n+1}).

Implicit R-K methods, on the other hand, require the solution of a coupled system
of equations.

Practical Ramifications
-----------------------

This unified notation means:

- One can keep *poses* on :math:`\SE(3)` by construction (no re-orthonormalization hacks).
- One keeps *spatial dynamics* in Featherstone form (fast, standard, and physically interpretable).
- One can swap integrators (RK4, Radau IIA, Gauss) while preserving the manifold structure.
- One's EKF propagation naturally uses tangent-space coordinates (algebra vectors) consistent with the manifold.

Note: :math:`\R^m` is also a Lie group
--------------------------------------

The additive group :math:`(\R^m,+)` is a Lie group with trivial exponential and
:math:`\dexp`. So the whole state space :math:`\SE(3)\times\R^m` can be viewed as a
direct-product Lie group. In implementation, :math:`y` is still kept in plain
vector form because its Lie machinery is trivial.

Appendix
--------

Hat/Vee Operators: From a Twist in :math:`\mathbb{R}^6` to :math:`\Lie{se}(3)`
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This appendix explains the “hat” operator :math:`(\cdot)^\wedge` used throughout
the document to map a 6D spatial motion vector (twist) into the Lie algebra
:math:`\Lie{se}(3)`, and its inverse “vee” operator :math:`(\cdot)^\vee` mapping
back to coordinates in :math:`\R^6`.

The hat map on :math:`\R^3`: :math:`\omega \mapsto [\omega]_\times`
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

For :math:`\omega = (\omega_x,\omega_y,\omega_z)^\top \in \R^3`, define the
skew-symmetric matrix

.. math::

   [\omega]_\times
   \;=\;
   \begin{bmatrix}
   0 & -\omega_z & \omega_y\\
   \omega_z & 0 & -\omega_x\\
   -\omega_y & \omega_x & 0
   \end{bmatrix}
   \in \Lie{so}(3).

It is characterized by the identity

.. math::

   [\omega]_\times\,a \;=\; \omega \times a,
   \qquad \forall\, a\in\R^3,

so “multiplication by :math:`[\omega]_\times`” is exactly the linear map “cross
with :math:`\omega`.”

The hat map on :math:`\R^6`: :math:`(\omega,v)\mapsto \Lie{se}(3)`
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

A twist (spatial motion vector) is represented in coordinates by

.. math::

   \twist
   =
   \begin{bmatrix}\omega\\ v\end{bmatrix}\in\R^6,
   \qquad \omega,v\in\R^3.

The hat map :math:`(\cdot)^\wedge:\R^6\to \Lie{se}(3)` packs these coordinates into
the standard matrix representation of :math:`\Lie{se}(3)`:

.. math::

   \twist^\wedge
   \;=\;
   \begin{bmatrix}
   [\omega]_\times & v\\
   0 & 0
   \end{bmatrix}_{4x4}
   \in \Lie{se}(3).

Thus, :math:`\omega` occupies the rotational (skew) block, and :math:`v` occupies
the translational column.

Why this mapping is natural: recovering the kinematics
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Let the pose be

.. math::

   g
   =
   \begin{bmatrix}
   R & p\\
   0 & 1
   \end{bmatrix}
   \in \SE(3),
   \qquad R\in\SO(3),\ p\in\R^3.

Using *left-trivialized* (body) kinematics,

.. math::

   \dot g = g\,\twist^\wedge,

and substituting :math:`\twist^\wedge` gives

.. math::

   \dot g
   =
   \begin{bmatrix}
   R & p\\
   0 & 1
   \end{bmatrix}
   \begin{bmatrix}
   [\omega]_\times & v\\
   0 & 0
   \end{bmatrix}
   =
   \begin{bmatrix}
   R[\omega]_\times & Rv\\
   0 & 0
   \end{bmatrix}.

Equating blocks yields the familiar rigid-body kinematics

.. math::

   \dot R = R[\omega]_\times,
   \qquad
   \dot p = Rv.

So in the left-trivialized convention, :math:`\omega` and :math:`v` are expressed
in the *body* frame, and are mapped into the inertial/world frame by
left-multiplication with :math:`R`.

The vee map: extracting coordinates back to :math:`\R^6`
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The inverse map :math:`(\cdot)^\vee:\Lie{se}(3)\to\R^6` simply unpacks the matrix
into its coordinate vector:

.. math::

   \left(
   \begin{bmatrix}
   [\omega]_\times & v\\
   0 & 0
   \end{bmatrix}
   \right)^\vee
   =
   \begin{bmatrix}\omega\\ v\end{bmatrix}.

(Here :math:`\omega` is recovered from the skew block via the standard
identification between :math:`\Lie{so}(3)` and :math:`\R^3`.)

Remark on conventions (body vs. spatial)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Equation :math:`\dot g = g\,\twist^\wedge` corresponds to a *body* twist
(left-trivialized velocity). If instead one uses a *spatial* twist
(right-trivialized velocity) expressed in world coordinates rather than body
coordinates, the kinematics take the form

.. math::

   \dot g = \twist^\wedge g,

and the coordinate interpretation of :math:`(\omega,v)` changes accordingly. If
the world frame is an inertial frame then it is an “inertial twist”. The document
adopts the left-trivialized convention to match the common “body” form used in
many Lie-group integrators and in Featherstone-style rigid-body dynamics.

