Appendices
==========

Algorithmic Differentiation
---------------------------
.. _appendix-ad:

Algorithmic Differentiation (AD) is a computational technique for evaluating
exact derivatives of functions implemented as computer programs. Unlike
symbolic differentiation, AD does not manipulate expressions, and unlike
finite differences, it introduces no truncation errors. AD applies the
chain rule to the sequence of elementary operations executed by the program.

Forward-Mode and Reverse-Mode AD
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Let :math:`f:\mathbb{R}^n \rightarrow \mathbb{R}^m` be evaluated by a
computational graph with :math:`L` operations.

**Forward-mode AD.** For a seed vector :math:`v \in \mathbb{R}^n`, forward-mode
computes

.. math::

   f'(x)\,v,

by propagating tangent values through the graph. Cost is proportional to
evaluating :math:`f` itself. Forward-mode is optimal when :math:`n` is small
and :math:`m` is large.

**Reverse-mode AD.** Reverse-mode computes

.. math::

   u^\top f'(x),

for any output seed :math:`u \in \mathbb{R}^m`, via a forward pass followed by
a backward pass (adjoints). It is optimal when :math:`n` is large and
:math:`m` is small, typical in optimal control, estimation, and flight
dynamics.

Practical Use with CppAD
~~~~~~~~~~~~~~~~~~~~~~~~

A typical workflow:

1. Wrap the dynamics :math:`f(x,u)` in CppAD types (e.g. ``AD<double>``).
2. Execute :math:`f` once to record the computational graph.
3. Request Jacobians or directional derivatives via CppAD.

The same graph can be reused to compute :math:`\mathbf{F}(x,u)` and
:math:`\mathbf{H}(x)` at every EKF step with low overhead.

Advantages over Finite Differences
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Finite differences approximate derivatives as

.. math::

   \frac{\partial f}{\partial x_i}
   \approx
   \frac{f(x + \epsilon e_i) - f(x)}{\epsilon},

which introduces truncation error and requires :math:`n+1` evaluations of
:math:`f` for an :math:`n`-dimensional system.

AD provides instead:

- machine-precision derivatives,
- deterministic runtime,
- no step-size parameter,
- reduced computational cost,
- exact sparsity patterns in :math:`\mathbf{F}` and :math:`\mathbf{H}`.

These properties are attractive for high-performance flight simulation,
optimal control, and GNC.


Functional Mock-up Units (FMUs)
-------------------------------
.. _appendix-fmu:

The Functional Mock-up Interface (FMI) standard defines a tool-independent
representation of dynamic models for co-simulation and model exchange.
An FMU (Functional Mock-up Unit) is a self-contained archive containing:

- model equations or compiled binaries,
- metadata (XML schema describing variables and interfaces),
- optional source code and resources,
- C-callable FMI functions.

FMUs allow flight-dynamics models, controllers, estimators, and environmental
models to be exchanged between tools without rewriting code.

FMI for Model Exchange
~~~~~~~~~~~~~~~~~~~~~~

Under *FMI for Model Exchange*, the FMU exposes

.. math::

   \dot{x} = f(x, u),

and

.. math::

   y = h(x, u),

while the importing tool provides the numerical solver.

This is ideal for SVA-based rigid-body dynamics: the FMU remains
solver-agnostic and all integration details are external.

FMI for Co-Simulation
~~~~~~~~~~~~~~~~~~~~~

Under *FMI for Co-Simulation*, the FMU carries its own integrator and
implements a one-step map

.. math::

   x(t_{k+1}) =
   \Phi\left( x(t_k), u(t_k), \Delta t \right),

where :math:`\Phi` is internal. This is useful when

- exact reproducibility is required,
- internal stiffness or fast rotations are handled by a specific solver,
- the FMU is used in real-time HIL setups.

SVA-based dynamics benefit from co-simulation for high update rates
(1–10 kHz) inside embedded GNC pipelines.

Mapping SVA Dynamics to FMU Variables
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

A typical FMU variable set for flight simulation includes:

- **States**:

  - :math:`\mathbf{p}_W` (position),
  - :math:`q_{WB}` (quaternion),
  - :math:`\boldsymbol{\omega}_B`, :math:`\mathbf{v}_B` (spatial motion),
  - :math:`m` (mass).

- **Inputs**:

  - thrust magnitude :math:`T`,
  - control torque :math:`\boldsymbol{\tau}_{ctrl,B}`,
  - wind velocity :math:`\mathbf{v}_{wind,W}`,
  - atmospheric properties (:math:`\rho`, temperature).

- **Outputs**:

  - aerodynamic forces,
  - acceleration in body or inertial frame,
  - energy or momentum terms,
  - sensor proxy signals.

Exporting the Model as an FMU
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Exporting the SVA ODE model requires:

1. A C/C++ function implementing :math:`f(x,u)` and :math:`h(x,u)`.
2. FMI-compatible wrappers (e.g. ``fmi2_doStep``, ``fmi2_getDerivatives``).
3. A ``modelDescription.xml`` file.
4. Packaging everything into a ``.fmu`` archive.

Existing tools include FMI Library, FMPy, FMIKit-Simulink, FMU4cpp,
or custom CMake-based exporters.

Using FMUs in GNC Pipelines
~~~~~~~~~~~~~~~~~~~~~~~~~~~

FMUs enable modular architectures where:

- a **flight dynamics** FMU provides SVA-based rigid-body motion,
- the **controller** is implemented in Simulink, C++, or Python,
- the **sensor suite** is represented by separate FMUs,
- the **environment** (wind, atmosphere) is another FMU.

Advantages:

- tool-independent model exchange,
- deterministic and repeatable simulations,
- plug-and-play substitution of controllers or subsystems,
- compatibility with digital twins and HIL environments.

Because the SVA implementation is algebraic and smooth, the resulting FMUs
are numerically stable and well suited to high-fidelity simulation,
optimization, and state estimation.
