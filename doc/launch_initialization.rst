Launch-Site Initialization and Frame Chain
==========================================

Frame Chain Overview
--------------------

The launch configuration uses the following frame chain:

.. math::

   W \rightarrow E \rightarrow U \rightarrow R \rightarrow B

where

- :math:`W`: ECI/inertial frame,
- :math:`E`: ECEF frame,
- :math:`U`: local NEU frame,
- :math:`R`: launch rail frame,
- :math:`B`: rocket body frame.

A schematic of this frame chain is:

.. figure:: _static/frame_chain_launch.png
   :alt: Frame chain for launch-site initialization
   :align: center

   Frame chain ECI (:math:`W`) → ECEF (:math:`E`) → NEU (:math:`U`)
   → rail (:math:`R`) → body (:math:`B`).


WGS–84 Geodetic Coordinates to ECEF
-----------------------------------

For geodetic latitude :math:`\varphi`, longitude :math:`\lambda`, and
height :math:`h` (WGS–84), the prime vertical radius of curvature is

.. math::

   N(\varphi)
   =
   \frac{a}{\sqrt{1 - e^2 \sin^2\varphi}},

where :math:`a` is the semi-major axis and :math:`e` is the first
eccentricity of WGS–84.

The ECEF coordinates of the launch site:

.. math::

   \begin{aligned}
   x_E &= (N(\varphi) + h)\cos\varphi \cos\lambda, \\
   y_E &= (N(\varphi) + h)\cos\varphi \sin\lambda, \\
   z_E &= \bigl((1-e^2)N(\varphi) + h\bigr)\sin\varphi.
   \end{aligned}

Thus,

.. math::

   {}^{E}\mathbf{r}_{\text{site}}
   =
   \begin{bmatrix}
     x_E \\ y_E \\ z_E
   \end{bmatrix}.


Local NEU Frame at the Launch Site
----------------------------------

The rotation from ECEF to local NEU is

.. math::

   {}^{U}\!R_{E}
   =
   \begin{bmatrix}
     -\sin\varphi \cos\lambda & -\sin\varphi \sin\lambda & \cos\varphi \\
     -\sin\lambda             &  \cos\lambda             & 0           \\
      \cos\varphi \cos\lambda &  \cos\varphi \sin\lambda & \sin\varphi
   \end{bmatrix}.

Each row is a NEU unit vector expressed in ECEF:

.. math::

   \hat{n}_U = {}^{U}\!R_{E}(1,:),\quad
   \hat{e}_U = {}^{U}\!R_{E}(2,:),\quad
   \hat{u}_U = {}^{U}\!R_{E}(3,:).

The inverse (NEU → ECEF) is

.. math::

   {}^{E}\!R_{U}
   = ({}^{U}\!R_{E})^{\mathsf{T}}.


ECEF to ECI Transformation
--------------------------

Let :math:`\theta_g(t)` be the Greenwich sidereal angle at time :math:`t`.
The ECEF → ECI rotation is

.. math::

   {}^{W}\!R_{E}(t)
   =
   \begin{bmatrix}
     \cos\theta_g(t) & -\sin\theta_g(t) & 0 \\
     \sin\theta_g(t) &  \cos\theta_g(t) & 0 \\
     0               &  0               & 1
   \end{bmatrix}.

The NEU frame in ECI coordinates is

.. math::

   {}^{W}\!R_{U}(t)
   =
   {}^{W}\!R_{E}(t)\; {}^{E}\!R_{U},

and the corresponding quaternion is

.. math::

   {}^{W}\!q_{U}(t) = \mathrm{quat}\bigl( {}^{W}\!R_{U}(t) \bigr).

Position mapping:

.. math::

   \mathbf{p}_E
   =
   \begin{bmatrix}
     x_E \\ y_E \\ z_E
   \end{bmatrix},
   \qquad
   \mathbf{p}_W(t)
   =
   {}^{W}\!R_{E}(t)\,\mathbf{p}_E.


Position and Velocity in the ECI Frame
--------------------------------------

Position in ECEF (from WGS–84) is :math:`\mathbf{p}_E` as above.

Position in ECI:

.. math::

   \mathbf{p}_W(t)
   =
   {}^{W}\!R_{E}(t)\,\mathbf{p}_E.

Earth’s rotation vector (in ECI):

.. math::

   \boldsymbol{\omega}_\oplus
   =
   \begin{bmatrix}
     0 \\ 0 \\ \omega_\oplus
   \end{bmatrix},
   \qquad
   \omega_\oplus \approx 7.2921159\times 10^{-5}\ \mathrm{rad/s}.

Velocity in ECI (for a fixed site on rotating Earth, no motion in ECEF):

.. math::

   \mathbf{v}_W(t)
   =
   \boldsymbol{\omega}_\oplus \times \mathbf{p}_W(t).

Explicitly,

.. math::

   \mathbf{v}_W(t)
   =
   \begin{bmatrix}
     -\omega_\oplus\, y_W(t) \\
      \omega_\oplus\, x_W(t) \\
      0
   \end{bmatrix},
   \quad
   \mathbf{p}_W(t)
   =
   \begin{bmatrix}
     x_W(t) \\ y_W(t) \\ z_W(t)
   \end{bmatrix}.


Rail Direction from Azimuth and Elevation
-----------------------------------------

Let :math:`\psi` be the rail azimuth (from North toward East) and
:math:`\theta` be the rail elevation (from the horizontal plane).

Rail direction in NEU:

.. math::

   \hat{d}_{R,U}
   =
   \begin{bmatrix}
     \cos\theta \cos\psi \\
     \cos\theta \sin\psi \\
     \sin\theta
   \end{bmatrix}.

Transform to ECEF and ECI:

.. math::

   \hat{d}_{R,E} = {}^{E}\!R_{U}\,\hat{d}_{R,U},
   \qquad
   \hat{d}_{R,W}(t) = {}^{W}\!R_{E}(t)\,\hat{d}_{R,E}.

Set the rail frame :math:`R` axis:

.. math::

   \hat{z}_R = \hat{d}_{R,W}.


Construction of the Rail Frame
------------------------------

Let :math:`\hat{e}_U = [0,1,0]^\mathsf{T}` be East in NEU. Push it to ECI:

.. math::

   \hat{e}_W(t)
   =
   {}^{W}\!R_{E}(t)\;{}^{E}\!R_{U}\;\hat{e}_U.

Compute rail :math:`x`-axis:

.. math::

   \hat{x}_R
   =
   \frac{
     \hat{e}_W(t)
     - (\hat{e}_W(t)\cdot \hat{z}_R)\,\hat{z}_R
   }{
     \left\|\hat{e}_W(t)
     - (\hat{e}_W(t)\cdot \hat{z}_R)\,\hat{z}_R\right\|
   }.

Then

.. math::

   \hat{y}_R
   = \hat{z}_R \times \hat{x}_R.

Finally,

.. math::

   {}^{W}\!R_{R}
   =
   \begin{bmatrix}
     \hat{x}_R & \hat{y}_R & \hat{z}_R
   \end{bmatrix},
   \qquad
   {}^{W}\!q_{R} = \mathrm{quat}({}^{W}\!R_{R}).


Body Orientation on the Launch Rail
-----------------------------------

Let :math:`\phi_0` be the body roll about the rail axis. The rotation from
rail frame :math:`R` to body frame :math:`B` is

.. math::

   {}^{R}\!q_{B}
   =
   \begin{bmatrix}
     \cos(\phi_0/2) \\
     \hat{z}_R \,\sin(\phi_0/2)
   \end{bmatrix}.


Final Quaternion Mapping (ECI → Body)
-------------------------------------

The final ECI-to-body quaternion is

.. math::

   \boxed{
   {}^{W}\!q_{B}
   =
   {}^{W}\!q_{U}(t_0)
   \otimes
   {}^{U}\!q_{R}
   \otimes
   {}^{R}\!q_{B}.
   }

Equivalently, in rotation matrices:

.. math::

   {}^{W}\!R_{B}
   =
   {}^{W}\!R_{U}(t_0)\;
   {}^{U}\!R_{R}\;
   {}^{R}\!R_{B}.
