.. _group-exp-log:

Group Exponential and Logarithm
===============================

.. contents::
   :local:
   :depth: 2

Introduction
------------

The group exponential and logarithm are best understood as the *curved-space*
versions of operations you already know on :math:`\mathbb{R}` and
:math:`\mathbb{R}^n`:

- In :math:`\mathbb{R}`, “integrate a constant velocity for 1 second” is just adding a number.
- In :math:`\mathbb{R}^n`, it is adding a vector.
- On a Lie group :math:`G`, it is multiplying by a group element obtained from an algebra element via :math:`\Exp`.

The Lie algebra :math:`\mathfrak{g}` plays the role of a *linearization* of
:math:`G` at the identity: it is the “tangent vector space” where one can do
vector addition, scaling, and linear algebra.

Number line example: :math:`(\mathbb{R}, +)`
--------------------------------------------

Consider the additive group :math:`G=(\mathbb{R},+)`:

.. math::

   e=0,\qquad \mathfrak{g} \cong \mathbb{R}.

The left-invariant ODE that defines the group exponential is

.. math::

   \dot x(t)=v,\qquad x(0)=0,

where :math:`v\in\mathbb{R}` is a constant (an element of the Lie algebra). The
solution is

.. math::

   x(t)=tv.

Thus the group exponential is simply

.. math::

   \Exp(v) = v,

because “moving from :math:`0` by velocity :math:`v` for unit time” lands at
:math:`v`.

Logarithm
^^^^^^^^^

The group logarithm is the inverse map, again the identity:

.. math::

   \Log(x)=x.

Composition vs Baker–Campbell–Hausdorff
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

On :math:`\mathbb{R}` we have

.. math::

   \Exp(v)+\Exp(w)=v+w=\Exp(v+w),

so there are *no* higher-order commutator corrections (the Lie bracket is
identically zero). This is the “flat” case.

Euclidean vector space: :math:`(\mathbb{R}^n, +)`
-------------------------------------------------

Now let :math:`G=(\mathbb{R}^n,+)`:

.. math::

   e=\bm{0},\qquad \mathfrak{g} \cong \mathbb{R}^n.

The defining ODE is

.. math::

   \dot x(t)=u,\qquad x(0)=0,\qquad u\in\mathbb{R}^n,

so :math:`x(t)=tu` and therefore

.. math::

   \Exp(u)=u,\qquad \Log(x)=x.

Interpretation for robotics/GNC
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

If state has an Euclidean part (position, velocity, mass, sensor biases),
those components already *live in a Lie group* (an abelian one). The reason we
still separate them from, say, :math:`\SO(3)` is practical:

- Euclidean parts update by addition: :math:`x^+ = x + \delta x`.
- Rotations update by group multiplication: :math:`R^+ = R\,\Exp(\delta\phi^\wedge)`.

So the “Runge–Kutta–Munthe-Kaas for product manifolds” view becomes: do additive
Runge–Kutta for the :math:`\mathbb{R}^n` factors and multiplicative Runge–Kutta
for the :math:`\SO(3)` factors, but *in one unified framework*.

A second number-line example: :math:`(\mathbb{R}_{>0}, \times)`
--------------------------------------------------------------

Now take the *multiplicative* group of positive real numbers:

.. math::

   G=(\mathbb{R}_{>0},\times),\qquad e=1.

Its Lie algebra is again :math:`\mathfrak{g}\cong \mathbb{R}` (a tangent line at
:math:`1`). The left-invariant ODE is

.. math::

   \dot a(t) = a(t)\,v,\qquad a(0)=1,

whose solution is

.. math::

   a(t)=e^{tv}.

Therefore the group exponential is the *ordinary* exponential:

.. math::

   \Exp(v)=e^{v}.

The group logarithm is the *ordinary* natural log:

.. math::

   \Log(a)=\ln(a).

Why this is a perfect analogy for rotations
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

For :math:`(\mathbb{R}_{>0},\times)`:

.. math::

   a^+ = a\,\Exp(v)

looks exactly like the rotation update

.. math::

   R^+ = R\,\Exp(\phi^\wedge),

except scalars commute and matrices generally do not.

Euclidean “counterpart” of :math:`\SO(3)`: small-angle approximation
--------------------------------------------------------------------

For :math:`R\in\SO(3)`, the Lie algebra :math:`\so(3)` is linear (a vector space).
For small :math:`\phi\in\mathbb{R}^3`,

.. math::

   \Exp(\phi^\wedge) \approx I + \phi^\wedge \quad (\|\phi\|\ll 1).

This mirrors the Euclidean idea “move by a small vector”:

.. math::

   x^+ = x + \delta x

while the group update is

.. math::

   R^+ = R\,\Exp(\delta\phi^\wedge)\approx R(I+\delta\phi^\wedge).

So *the Lie algebra element* :math:`\delta\phi` behaves like a Euclidean increment
(in the tangent space), and :math:`\Exp` wraps that increment back onto the
curved manifold.

Concrete paired examples: Euclidean vs group
--------------------------------------------

Translation in :math:`\mathbb{R}^3`
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Euclidean:

.. math::

   p^+ = p + \Delta p,\qquad p,\Delta p\in\mathbb{R}^3.

This is also a Lie group update in :math:`(\mathbb{R}^3,+)`:

.. math::

   p^+ = p \oplus \Exp(\Delta p),\qquad \Exp(\Delta p)=\Delta p,

where :math:`\oplus` is just :math:`+`.

Rotation in :math:`\SO(3)`
^^^^^^^^^^^^^^^^^^^^^^^^^^

Let :math:`R\in\SO(3)` and :math:`\delta\phi\in\mathbb{R}^3` (tangent increment):

.. math::

   R^+ = R\,\Exp(\delta\phi^\wedge).

If you want the corresponding tangent quantity from two rotations
:math:`R_1,R_2`:

.. math::

   \delta\phi^\wedge = \Log(R_1^\top R_2).

Compare the Euclidean counterpart:

.. math::

   \Delta x = x_2 - x_1.

So “difference” on a group is obtained by *relative element + log*:

.. math::

   \text{Euclidean: } x_2 - x_1
   \qquad \leftrightarrow \qquad
   \text{Group: } \Log(g_1^{-1}g_2).

Rigid pose in :math:`\SE(3)` vs Euclidean stacking
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Euclidean (naively stacking position and orientation parameters) is not closed
under addition in a physically consistent way. On :math:`\SE(3)`, a pose update is

.. math::

   T^+ = T\,\Exp(\xi^\wedge),

where :math:`\xi=(\rho,\phi)\in\mathbb{R}^6` is a twist (tangent increment). The
“difference” between two poses is

.. math::

   \xi^\wedge = \Log(T_1^{-1}T_2),

which is the pose-analogue of :math:`x_2-x_1`.

Summary: the dictionary
-----------------------

.. math::

   \begin{array}{c|c|c}
   \textbf{Concept} & \textbf{Euclidean } (\mathbb{R}^n,+) & \textbf{Lie group } G \\
   \hline
   \text{Identity} & 0 & e \\
   \text{Increment space} & \mathbb{R}^n & \mathfrak{g}=T_eG \\
   \text{Update} & x^+=x+\delta & g^+=g\,\Exp(\delta) \\
   \text{Difference} & \delta=x_2-x_1 & \delta=\Log(g_1^{-1}g_2) \\
   \text{``Addition'' of increments} & \delta_1+\delta_2 & \Log(\Exp(\delta_1)\Exp(\delta_2)) \\
   \end{array}

Key intuition
^^^^^^^^^^^^^

In Euclidean space, the exponential/log are trivial (identity). In multiplicative
positive reals, :math:`\Exp` and :math:`\Log` become the familiar scalar
:math:`\exp` and :math:`\ln`. For rotations and poses, :math:`\Exp` and :math:`\Log`
generalize these ideas to non-commuting matrix groups.
