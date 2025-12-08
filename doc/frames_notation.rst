Coordinate Frames and Basic Notation
====================================

Coordinate Frames
-----------------

We adopt the following reference frames for mathematical modeling:

- :math:`W`: a mathematically inertial frame (often chosen as ECI),
- :math:`I`: a physical Earth–Centered Inertial frame (ECI),
- :math:`E`: Earth–fixed rotating frame (ECEF).

Atmospheric velocity expressed in :math:`W` is

.. math::

   \mathbf{v}_{\text{air},W}
   =
   \boldsymbol{\omega}_\oplus \times \mathbf{r}_W,

where :math:`\boldsymbol{\omega}_\oplus` is Earth's rotation and
:math:`\mathbf{r}_W` is expressed in :math:`W`.

Frame Relationship Figure
-------------------------

.. figure:: _static/eci_ecef_frames.pdf
   :alt: Relationship between Earth-fixed frame E and inertial frame W
   :align: center

   Relationship between Earth-fixed frame :math:`E`, Earth's rotation
   :math:`\boldsymbol{\omega}_\oplus`, and the inertial/ECI frame
   :math:`W \equiv I`.


Notation Summary
----------------

- Position in inertial frame:

  .. math::

     \mathbf{p}_W \in \mathbb{R}^3.

- Orientation unit quaternion and associated rotation matrices between
  inertial and body frames:

  .. math::

     R_{WB} = \mathcal{R}(q_{WB}),
     \qquad
     R_{BW} = R_{WB}^\mathsf{T}.

- Spatial motion vector in body frame:

  .. math::

     \mathbf{v}_B
     =
     \begin{bmatrix}
       \boldsymbol{\omega}_B \\
       \mathbf{v}_B
     \end{bmatrix}
     \in \mathbb{R}^6.

- Spatial inertia (simple block form):

  .. math::

     \mathbf{I}_B(m)
     =
     \begin{bmatrix}
       \mathbf{J}_B(m) & \mathbf{0} \\
       \mathbf{0} & m \mathbf{I}_3
     \end{bmatrix},
     \qquad
     \mathbf{I}_B(m) \in \mathbb{R}^{6\times 6}.

- Cross operator :math:`\times` operating on spatial motion vectors, and
  its dual :math:`\times^*` operating on wrenches:

  .. math::

     \mathbf{v}_B \times
     =
     \begin{bmatrix}
       [\boldsymbol{\omega}_B]_\times & \mathbf{0} \\
       [\mathbf{v}_B]_\times & [\boldsymbol{\omega}_B]_\times
     \end{bmatrix},
     \quad
     \mathbf{v}_B \times^{*}
     =
     -(\mathbf{v}_B \times)^\mathsf{T}.
