.. _rkmk-product-manifolds:

RKMK Numerical Integration on Product Manifolds
===============================================

Lie Group × Euclidean Factors
-----------------------------

.. contents::
   :local:
   :depth: 2

.. rubric:: Abstract

Many flight-dynamics and GNC state vectors naturally live on *product manifolds*,
e.g., attitude on :math:`\SO(3)` (or unit quaternions) together with translational
states in :math:`\R^n`. Standard Runge–Kutta methods assume a vector space and can
break manifold constraints (e.g. orthonormality, unit norm). Runge–Kutta–Munthe-Kaas
(RKMK) methods lift the integration to the Lie algebra and update the group by an
exponential (or a retraction), achieving high order while preserving the manifold
structure.

These notes explain RKMK in a product-manifold setting, emphasizing the typical case
:math:`\SO(3)\times \R^n`, and how to consistently couple the Lie-group and Euclidean
factors.

Why product manifolds show up in GNC?
-------------------------------------

A common state layout is

.. math::

   x = (R,\, p,\, v,\, \omega,\, b,\, \dots)
   \in \SO(3)\times \R^3\times \R^3\times \R^3\times \R^m \times \cdots.

The *configuration* (attitude) is not a vector: :math:`R\in \SO(3)` satisfies
:math:`R^\top R = I` and :math:`\det(R)=1`. If we apply a vanilla Runge–Kutta scheme
directly to a matrix parameterization (or integrate a quaternion and re-normalize),
we risk:

- drift from the manifold (loss of orthonormality / unit norm),
- loss of accuracy order due to ad-hoc projection/normalization,
- inconsistency between continuous-time model and discrete-time propagation used by an
  EKF (Extended Kalman Filter).

RKMK addresses this by:

1. performing Runge–Kutta stages in the *Lie algebra* (a vector space),
2. mapping algebra increments back to the group via :math:`\exp` (or a retraction),
3. integrating Euclidean factors normally (addition).

Product manifolds and tangent spaces
------------------------------------

Definition
^^^^^^^^^^

A *product manifold* is :math:`M = M_1 \times \cdots \times M_k` where each
:math:`M_i` is a manifold. A point is :math:`x=(x_1,\dots,x_k)` with
:math:`x_i\in M_i`.

Tangent space of a product
^^^^^^^^^^^^^^^^^^^^^^^^^^

The tangent space factorizes:

.. math::

   T_{(x_1,\dots,x_k)}M \;\cong\; T_{x_1}M_1 \times \cdots \times T_{x_k}M_k.

So a vector field :math:`\dot x = F(x)` on :math:`M` splits into components

.. math::

   \dot x_i = F_i(x_1,\dots,x_k), \qquad i=1,\dots,k,

but *the components can still be coupled* through the shared dependence on all factors.

When factors are Lie groups
^^^^^^^^^^^^^^^^^^^^^^^^^^^

If :math:`G` is a Lie group, then each tangent space :math:`T_g G` can be
identified with the Lie algebra :math:`\Lie{g} = T_e G` via left translation:

.. math::

   T_g G \ni \dot g \quad\leftrightarrow\quad \xi = g^{-1}\dot g \in \Lie{g}.

For matrix Lie groups (like :math:`\SO(3)`), :math:`g^{-1} = g^\top` and
:math:`\xi` is a matrix in the algebra.

For a *direct-product Lie group* :math:`G = G_1 \times G_2`, the Lie algebra is the
direct sum

.. math::

   \Lie{g} = \Lie{g}_1 \oplus \Lie{g}_2,

and :math:`\exp_G` acts component-wise:

.. math::

   \exp_G(\xi_1,\xi_2) = (\exp_{G_1}(\xi_1),\exp_{G_2}(\xi_2)).

In practice, most GNC states look like a Lie group factor :math:`G` times Euclidean
:math:`\R^n` (which is itself a Lie group under addition).

Dynamics on :math:`G\times \R^n`
--------------------------------

Let :math:`M = G \times \R^n`, with state :math:`x=(g,r)`. A broad class of coupled
dynamics is:

.. math::

   \dot g = g\, \Omega(g,r), \qquad
   \dot r = f(g,r),

where :math:`\Omega(g,r)\in \Lie{g}` is the left-trivialized “angular” rate (for
:math:`G=\SO(3)` this corresponds to body angular velocity), and
:math:`f:\,G\times \R^n\to \R^n` is the Euclidean part.

**Key idea:** :math:`\dot g` lives on the group; :math:`\dot r` lives in a vector
space. RKMK integrates them together without leaving :math:`G`.

Retractions and the exponential map
-----------------------------------

Exponential on the group
^^^^^^^^^^^^^^^^^^^^^^^^

For a Lie group :math:`G`, the exponential map

.. math::

   \exp:\ \Lie{g} \to G

maps an algebra element (a tangent vector at identity) to a group element.

Given a base point :math:`g_n\in G`, an increment :math:`\eta\in \Lie{g}` updates
the state as

.. math::

   g_{n+1} = g_n \exp(\eta).

This is a *retraction* based at :math:`g_n`.

Euclidean factor as a trivial Lie group
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

On :math:`\R^n`, the exponential is just addition:

.. math::

   r_{n+1} = r_n + \Delta r.

This matches the product-manifold view: we retract on :math:`G` by :math:`\exp`,
and on :math:`\R^n` by :math:`+`.

From classical RK to RKMK
-------------------------

Classical Runge–Kutta (RK) methods are defined for ODEs in a vector space:

.. math::

   \dot y = \phi(y),\quad y\in \R^m.

RKMK adapts an RK tableau :math:`(a_{ij}, b_i, c_i)` to Lie-group evolution by:

1. representing stage states as :math:`g_n \exp(\eta_i)` for algebra stage variables
   :math:`\eta_i`,
2. evaluating the group vector field in left-trivialized form :math:`\Omega(\cdot)`,
3. correcting for the nonlinear relationship between :math:`\eta` and :math:`g`
   using :math:`\dexp^{-1}`.

The :math:`\dexp` operator
^^^^^^^^^^^^^^^^^^^^^^^^^^

Define :math:`\eta(t)\in \Lie{g}` by writing the group trajectory as

.. math::

   g(t) = g_n \exp(\eta(t)).

Differentiate:

.. math::

   \dot g(t) = g_n \frac{d}{dt}\exp(\eta(t)).

Left-trivialize at :math:`g(t)`:

.. math::

   g(t)^{-1}\dot g(t) = \Omega(g(t), r(t)).

Define the differential of the exponential by its Lie series,

.. math::

   \dexp_{\eta}(u) = \sum_{k\ge 0}\frac{1}{(k+1)!}\,\ad_{\eta}^{\,k}(u),

for which the standard identity is *right*-trivialized:

.. math::

   \Big(\tfrac{d}{dt}\exp(\eta)\Big)\exp(-\eta) = \dexp_{\eta}(\dot\eta).

Our field is left-trivialized, so we must move the correction across the group
element.  Using :math:`\Ad_{\exp(-\eta)}\dexp_{\eta} = \dexp_{-\eta}`,

.. math::

   \exp(-\eta)\Big(\tfrac{d}{dt}\exp(\eta)\Big) = \dexp_{-\eta}(\dot\eta),

hence

.. math::

   g(t)^{-1}\dot g(t) = \dexp_{-\eta(t)}(\dot \eta(t)),

and therefore

.. math::

   \dot \eta(t) = \dexp_{-\eta(t)}^{-1}\!\big(\Omega(g_n\exp(\eta(t)), r(t))\big).

This is the core: it converts the *group* ODE into an *algebra* ODE (a vector
space), where we can apply RK.

In Bernoulli-series form,

.. math::

   \dexp_{-\eta}^{-1}
   = \sum_{k\ge 0}\frac{B_k}{k!}\,\ad_{-\eta}^{\,k}
   = I + \tfrac{1}{2}\ad_{\eta} + \tfrac{1}{12}\ad_{\eta}^{2}
     - \tfrac{1}{720}\ad_{\eta}^{4} + \mathcal{O}(\ad_{\eta}^{6}),

where the :math:`\ad^{3}` term vanishes because :math:`B_3 = 0`.

.. warning::

   **The sign of the argument is load-bearing.**  The series is most often
   tabulated for the *right*-trivialized convention
   :math:`\dot g = \hat\xi\,g`, where the correction is
   :math:`\dexp_{+\eta}^{-1} = I - \tfrac12\ad_\eta + \tfrac1{12}\ad_\eta^2 - \cdots`.
   Body-frame flight dynamics is left-trivialized
   (:math:`\dot g = g\,\hat\xi`), so the correct correction is evaluated at
   :math:`-\eta` and the :math:`\mathcal{O}(\ad)` term changes sign.

   Using the wrong sign does *not* produce an obviously broken simulation.  The
   group element still stays exactly on the manifold, the Newton iteration
   still converges, and constant-twist motion is still integrated exactly
   (because :math:`\ad_\eta\eta = 0`, so the erroneous term vanishes).  What it
   does is silently reduce the method to **order 2 for any Butcher tableau**.
   The defect is observable only in a convergence study on a problem whose body
   twist actually varies in time --- for example a torque-free *asymmetric*
   body.  See :ref:`rkmk_order_verification`.

RKMK on :math:`G\times \R^n`: coupled stage equations
-----------------------------------------------------

Let step size be :math:`h` and tableau :math:`(a_{ij},b_i,c_i)` have :math:`s` stages.
We propagate from :math:`(g_n,r_n)` to :math:`(g_{n+1},r_{n+1})`.

Stage reconstruction on the product manifold
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

We maintain:

.. math::

   \eta_i \in \Lie{g},\qquad \rho_i \in \R^n,

and define stage states

.. math::

   g_i = g_n \exp(\eta_i),\qquad r_i = r_n + \rho_i.

Stage derivatives
^^^^^^^^^^^^^^^^^

Compute:

.. math::

   K_i = \dexp_{-\eta_i}^{-1}\!\left(\Omega(g_i,r_i)\right)\in \Lie{g}, \qquad
   k_i = f(g_i,r_i)\in \R^n.

Stage accumulation
^^^^^^^^^^^^^^^^^^

For an *explicit* R-K tableau, we set

.. math::

   \eta_i = h \sum_{j=1}^{i-1} a_{ij} K_j,\qquad
   \rho_i = h \sum_{j=1}^{i-1} a_{ij} k_j.

For implicit tableaus, these become coupled non-linear equations within the stages.

Step update
^^^^^^^^^^^

Finally,

.. math::

   \eta_{n+1} = h\sum_{i=1}^s b_i K_i,\qquad
   r_{n+1} = r_n + h\sum_{i=1}^s b_i k_i,\qquad
   g_{n+1} = g_n \exp(\eta_{n+1}).

**Interpretation on a product manifold:**

- The group factor is advanced by a *single* exponential at the end (plus exponentials
  inside stages).
- The Euclidean factor is advanced by the same R-K weights, but with ordinary addition.
- Coupling enters only through the evaluations :math:`\Omega(g_i,r_i)` and :math:`f(g_i,r_i)`.

Concrete special case: :math:`\SO(3)\times \R^n`
------------------------------------------------

SO(3) algebra and the hat operator
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Identify :math:`\Lie{so}(3)\cong \R^3` via the hat map:

.. math::

   \phi \in \R^3
   \quad\mapsto\quad
   [\phi]_\times \in \Lie{so}(3),
   \qquad
   [\phi]_\times =
   \begin{bmatrix}
   0 & -\phi_3 & \phi_2\\
   \phi_3 & 0 & -\phi_1\\
   -\phi_2 & \phi_1 & 0
   \end{bmatrix}.

Write :math:`\eta \equiv \phi` (vector form) and :math:`\exp(\eta) \equiv \Exp([\phi]_\times)`.

Exponential map on SO(3)
^^^^^^^^^^^^^^^^^^^^^^^^

Let :math:`\theta=\norm{\phi}`, :math:`A=[\phi]_\times`. Then Rodrigues' formula:

.. math::

   \Exp(A) = I + \frac{\sin\theta}{\theta}A + \frac{1-\cos\theta}{\theta^2}A^2,

with the usual smooth limits as :math:`\theta\to 0` (use series to avoid division by
small :math:`\theta`).

Left Jacobian and :math:`\dexp^{-1}` on SO(3)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

On :math:`\SO(3)` the operator :math:`\dexp_{\phi}` is the *left Jacobian*
:math:`J_\ell(\phi)`, which is also the matrix appearing in the translational
block of the :math:`\SE(3)` exponential:

.. math::

   \dexp_{\phi}(u) \equiv J_\ell(\phi)\,u,\qquad
   J_\ell(\phi) = I
   + \frac{1-\cos\theta}{\theta^2}[\phi]_\times
   + \frac{\theta-\sin\theta}{\theta^3}[\phi]_\times^2 .

What the RKMK stage correction needs, however, is :math:`\dexp_{-\phi}^{-1}`
(see the derivation above), and

.. math::

   \dexp_{-\phi} = J_\ell(-\phi) = J_r(\phi) = I
   - \frac{1-\cos\theta}{\theta^2}[\phi]_\times
   + \frac{\theta-\sin\theta}{\theta^3}[\phi]_\times^2,

the *right* Jacobian.  Its inverse is the quantity actually evaluated in code:

.. math::

   \dexp^{-1}_{-\phi} = J_r(\phi)^{-1} =
   I + \frac{1}{2}[\phi]_\times
   + \left(\frac{1}{\theta^2} - \frac{1+\cos\theta}{2\theta\sin\theta}\right)[\phi]_\times^2,
   \qquad \theta \not\approx 0 .

For small :math:`\theta`, use the series expansion

.. math::

   J_r(\phi)^{-1} \approx
   I + \frac{1}{2}[\phi]_\times + \frac{1}{12}[\phi]_\times^2 + \mathcal{O}(\theta^3).

This is often the most AD-friendly route: branch on :math:`\theta^2` and use series
near zero.

.. note::

   The :math:`+\tfrac12[\phi]_\times` here is the :math:`\SO(3)` instance of the
   :math:`+\tfrac12\ad_\eta` term in the Bernoulli series above.  If a derivation
   yields :math:`-\tfrac12[\phi]_\times` for a body-frame (left-trivialized)
   field, the trivialization convention has been mixed up.

.. _rkmk_order_verification:

Verifying the order in practice
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Convergence order cannot be verified on a torque-free *symmetric* body.  For
:math:`I_{xx}=I_{yy}=I_{zz}` and no external wrench, Euler's equations give
:math:`\dot\omega = 0`, so the body twist is constant and the exact flow is the
one-parameter subgroup :math:`g(t) = g_0\exp(t\xi)`.  Every RKMK method
reproduces that *exactly*, at every step size, regardless of its classical
order --- the errors sit at machine precision and the log-log slope is
meaningless noise.

A usable test problem must have a genuinely time-varying twist.  Aetherion uses
a torque-free **asymmetric** body (:math:`I_{xx}`, :math:`I_{yy}`,
:math:`I_{zz}` all distinct, so :math:`\omega\times I\omega \neq 0`) with a
non-zero initial body velocity, integrated against a reference computed at a
step size 50 times smaller than the finest tested step and cross-validated
between two independent integrators.  Measured mean log-log slopes:

.. list-table::
   :header-rows: 1
   :widths: 45 20 20

   * - Integrator
     - Attitude
     - Position
   * - Radau IIA RKMK (3-stage, order 5)
     - 5.00
     - 5.00
   * - Explicit RK4 RKMK (4-stage, order 4)
     - 3.91
     - 3.98

Both checks are regression tests
(:file:`tests/RigidBody/test_ConvergenceOrder.cpp`): the exactness property on
the symmetric sphere, and the measured order on the asymmetric body.

Typical coupled state example
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Let :math:`(R, z)\in \SO(3)\times \R^n` and define

.. math::

   \dot R = R[\omega(R,z)]_\times,\qquad \dot z = f(R,z).

Then in RKMK you compute stages:

.. math::

   K_i = J(\phi_i)^{-1}\,\omega(R_i,z_i),

with :math:`R_i = R_n \Exp([\phi_i]_\times)`, :math:`z_i = z_n + \rho_i`, and the same
R-K accumulation rules.

Note :math:`\R^n` is also a Lie group
-------------------------------------

:math:`(\R^n,+)` is a Lie group with Lie algebra :math:`\R^n`, exponential
:math:`\exp(v)=v` (identity), and group action by addition. So *in principle* you
can treat the whole state as a Lie group element in a direct product
:math:`G\times \R^n`.

In practice, you still keep the Euclidean part in vector form because:

- its exponential and :math:`\dexp` are trivial,
- you avoid unnecessary machinery,
- it keeps Jacobians/simple linear algebra straightforward.

RKMK on product manifolds is exactly this: nontrivial Lie factors get :math:`\exp`
and :math:`\dexp^{-1}`; Euclidean factors reduce to standard R-K.

Explicit example: RK4-RKMK on :math:`G\times \R^n`
--------------------------------------------------

Using classical RK4 tableau, stages are explicit. A compact description:

**Given** :math:`(g_n,r_n)` and step :math:`h`.

.. math::

   \eta_1 = 0,\quad \rho_1=0, \qquad
   (g_1,r_1) = (g_n,r_n),

.. math::

   K_1 = \dexp_{-\eta_1}^{-1}\!\left(\Omega(g_1,r_1)\right),\quad
   k_1 = f(g_1,r_1),

.. math::

   \eta_2 = \tfrac{h}{2}K_1,\quad \rho_2=\tfrac{h}{2}k_1,\qquad
   (g_2,r_2) = (g_n\exp(\eta_2),\, r_n+\rho_2),

.. math::

   K_2 = \dexp_{-\eta_2}^{-1}\!\left(\Omega(g_2,r_2)\right),\quad
   k_2 = f(g_2,r_2),

.. math::

   \eta_3 = \tfrac{h}{2}K_2,\quad \rho_3=\tfrac{h}{2}k_2,\qquad
   (g_3,r_3) = (g_n\exp(\eta_3),\, r_n+\rho_3),

.. math::

   K_3 = \dexp_{-\eta_3}^{-1}\!\left(\Omega(g_3,r_3)\right),\quad
   k_3 = f(g_3,r_3),

.. math::

   \eta_4 = hK_3,\quad \rho_4=h k_3,\qquad
   (g_4,r_4) = (g_n\exp(\eta_4),\, r_n+\rho_4),

.. math::

   K_4 = \dexp_{-\eta_4}^{-1}\!\left(\Omega(g_4,r_4)\right),\quad
   k_4 = f(g_4,r_4).

Then update:

.. math::

   \eta_{n+1} = \frac{h}{6}(K_1+2K_2+2K_3+K_4),\qquad
   r_{n+1} = r_n + \frac{h}{6}(k_1+2k_2+2k_3+k_4),

.. math::

   g_{n+1} = g_n \exp(\eta_{n+1}).

Implicit RKMK on product manifolds (stiff dynamics)
---------------------------------------------------

When dynamics are stiff (e.g. strong aerodynamic damping, tight actuator models,
flexible modes, or coupled constraints), implicit R-K methods (Radau IIA,
Gauss–Legendre) are attractive.

In implicit RKMK, the stage equations

.. math::

   \eta_i = h\sum_{j=1}^s a_{ij}K_j,\qquad
   \rho_i = h\sum_{j=1}^s a_{ij}k_j

are solved simultaneously with

.. math::

   K_i = \dexp_{-\eta_i}^{-1}\!\left(\Omega(g_i,r_i)\right),\qquad
   k_i = f(g_i,r_i).

This forms a nonlinear system in :math:`\{\eta_i,\rho_i\}_{i=1}^s`.

Newton structure on a product manifold
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Your unknown vector is a stacked Euclidean vector consisting of:

- Lie algebra coordinates for each :math:`\eta_i` (e.g. :math:`\R^3` per :math:`\SO(3)` factor),
- Euclidean increments :math:`\rho_i\in\R^n`.

The nonlinearities enter through :math:`\exp(\eta_i)` and :math:`\dexp^{-1}_{-\eta_i}`
(plus whatever nonlinearities are in :math:`\Omega` and :math:`f`). This is exactly
where algorithmic differentiation (e.g. CppAD) is helpful.

Quaternion normalization vs. Lie-group integration
--------------------------------------------------

If you represent attitude by a quaternion :math:`q\in S^3` and integrate
:math:`\dot q = \tfrac12 q\otimes \omega` with RK, you often re-normalize:
:math:`q \leftarrow q/\norm{q}`.

This is a projection back to :math:`S^3`. It can work well, but:

- the normalization is an extra nonlinear operation not accounted for in R-K order conditions,
- it can reduce the global order (especially for higher-order schemes) or complicate consistent linearization,
- the EKF propagation Jacobians become sensitive to the chosen “projection” step.

RKMK instead updates by a group exponential (or a retraction) and stays on the
manifold by construction:

.. math::

   R_{n+1} = R_n\Exp([\phi]_\times),\quad\text{or}\quad
   q_{n+1} = q_n \otimes \exp_{S^3}(\tfrac12\phi).

That tends to be cleaner for both accuracy and filter consistency.

Summary for :math:`G\times\R^n` implementations
-----------------------------------------------

1. Write group dynamics in left-trivialized form: :math:`\dot g = g\,\Omega(g,r)`.
2. Choose a base R-K tableau (explicit or implicit).
3. In each stage, reconstruct :math:`(g_i,r_i)` via :math:`(g_n\exp(\eta_i),\, r_n+\rho_i)`.
4. Evaluate :math:`\Omega(g_i,r_i)` and :math:`f(g_i,r_i)`.
5. Convert the group rate to the algebraic rate with :math:`\dexp^{-1}_{-\eta_i}`.
6. Accumulate stages using R-K coefficients.
7. Update :math:`g` with a final :math:`\exp(\cdot)` and :math:`r` with a final weighted sum.
8. For :math:`\SO(3)`, implement stable :math:`\Exp`, :math:`J(\phi)`, :math:`J(\phi)^{-1}` with small-angle series.

Appendix
--------

Useful SO(3) identities
^^^^^^^^^^^^^^^^^^^^^^^

For :math:`\phi\in\R^3` and :math:`A=[\phi]_\times`:

.. math::

   A^2 = \phi\phi^\top - \theta^2 I,\qquad \theta=\norm{\phi}.

These help implement Rodrigues formula and Jacobians efficiently.


