Kinematics, Forces, and Spatial Dynamics
========================================

Kinematics
----------

Position
~~~~~~~~

.. math::

   \dot{\mathbf{p}}_W
   = R_{WB}\,\mathbf{v}_B.

Orientation
~~~~~~~~~~~

Angular velocity in the inertial frame relates to angular velocity in the
body frame as

.. math::

   \boldsymbol{\omega}_W
   = R_{WB}\,\boldsymbol{\omega}_B.

Quaternion kinematics:

.. math::

   \dot{q}_{WB}
   =
   \frac{1}{2}
   \begin{bmatrix}
     0 \\ \boldsymbol{\omega}_W
   \end{bmatrix}
   \otimes q_{WB}.


Forces
------

Gravity
~~~~~~~

.. math::

   \mathbf{F}_{g,W} = m\,\mathbf{g}_W(\mathbf{p}_W),

.. math::

   \mathbf{F}_{g,B} = R_{BW}\,\mathbf{F}_{g,W},

.. math::

   \mathbf{f}_{g,B}
   =
   \begin{bmatrix}
     0 \\ \mathbf{F}_{g,B}
   \end{bmatrix}.


Rotating Atmosphere and Aerodynamics
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Earth rotation:

.. math::

   \boldsymbol{\omega}_\oplus
   =
   \begin{bmatrix}
     0 \\ 0 \\ \omega_\oplus
   \end{bmatrix}_W.

Atmospheric velocity:

.. math::

   \mathbf{v}_{air,W}
   = \boldsymbol{\omega}_\oplus \times \mathbf{p}_W.

Relative airspeed:

.. math::

   \mathbf{v}_W = R_{WB}\mathbf{v}_B,

.. math::

   \mathbf{v}_{rel,W}
   =
   \mathbf{v}_W - \mathbf{v}_{air,W} - \mathbf{v}_{wind,W}.

Aerodynamic drag:

.. math::

   \mathbf{F}_{aero,W}
   =
   -\frac{1}{2}\rho C_D A
     \|\mathbf{v}_{rel,W}\|\,
     \mathbf{v}_{rel,W}.

Transform to body frame:

.. math::

   \mathbf{F}_{aero,B}
   = R_{BW}\mathbf{F}_{aero,W},

.. math::

   \mathbf{f}_{aero,B}
   =
   \begin{bmatrix}
     0 \\ \mathbf{F}_{aero,B}
   \end{bmatrix}.


Thrust
~~~~~~

.. math::

   \mathbf{F}_{T,B} = T(t)\hat{\mathbf{t}}_B,

.. math::

   \mathbf{f}_{T,B}
   =
   \begin{bmatrix}
     0 \\ \mathbf{F}_{T,B}
   \end{bmatrix}.


Control Torques
~~~~~~~~~~~~~~~

.. math::

   \mathbf{f}_{ctrl,B}
   =
   \begin{bmatrix}
     \boldsymbol{\tau}_{ctrl,B}(t,x) \\ 0
   \end{bmatrix}.


Total Spatial Wrench
~~~~~~~~~~~~~~~~~~~~

.. math::

   \mathbf{f}_B
   =
   \mathbf{f}_{T,B}
   +
   \mathbf{f}_{g,B}
   +
   \mathbf{f}_{aero,B}
   +
   \mathbf{f}_{ctrl,B}.


Spatial Dynamics
----------------

Instead of a plain Newton–Euler formalism, six degree-of-freedom motion can
be expressed with a single equation using SVA (all terms in body frame):

.. math::

   \mathbf{I}_B(m)\mathbf{a}_B
   +
   \mathbf{v}_B \times^{*} \bigl(\mathbf{I}_B(m)\mathbf{v}_B\bigr)
   =
   \mathbf{f}_B.

Solving for acceleration:

.. math::

   \mathbf{a}_B
   =
   \mathbf{I}_B^{-1}
   \left[
     \mathbf{f}_B
     -
     \mathbf{v}_B \times^{*}(\mathbf{I}_B\mathbf{v}_B)
   \right].

Component form:

.. math::

   \mathbf{a}_B
   =
   \begin{bmatrix}
     \dot{\boldsymbol{\omega}}_B \\
     \dot{\mathbf{v}}_B
   \end{bmatrix}.


Mass Dynamics
-------------

.. math::

   \dot{m}(t) = -\dot{m}_{flow}(t),

.. math::

   T(t) = \dot{m}_{flow}(t)\,g_0\,I_{sp}.


Final ODE System
----------------

The system of ODEs is summarized in canonical form
:math:`\dot{x} = f(x)`:

.. math::

   \boxed{
   \begin{aligned}
     \dot{\mathbf{p}}_W &= R_{WB}\,\mathbf{v}_B, \\[6pt]
     \dot{q}_{WB} &=
       \frac{1}{2}
       \begin{bmatrix}
         0 \\
         R_{WB}\boldsymbol{\omega}_B
       \end{bmatrix}
       \otimes q_{WB}, \\[10pt]
     \begin{bmatrix}
       \dot{\boldsymbol{\omega}}_B \\
       \dot{\mathbf{v}}_B
     \end{bmatrix}
     &=
     \mathbf{I}_B^{-1}
     \left[
       \mathbf{f}_B
       - \mathbf{v}_B\times^{*} (\mathbf{I}_B\mathbf{v}_B)
     \right], \\[10pt]
     \dot{m} &= -\dot{m}_{flow}(t).
   \end{aligned}
   }
