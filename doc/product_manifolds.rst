.. _product-manifolds:

Product Manifolds
=================

.. contents::
   :local:
   :depth: 2

Product manifolds
-----------------

Definition
^^^^^^^^^^

Let :math:`(\mathcal{M}_1,\dots,\mathcal{M}_k)` be smooth manifolds. Their
*Cartesian product*

.. math::

   \mathcal{M} \;\coloneqq\; \mathcal{M}_1 \times \cdots \times \mathcal{M}_k
   \;=\;\{(x_1,\dots,x_k)\mid x_i\in\mathcal{M}_i\}

admits a natural smooth-manifold structure called the *product manifold*.

Charts and dimension
^^^^^^^^^^^^^^^^^^^^

If :math:`(U_i,\varphi_i)` is a chart on :math:`\mathcal{M}_i` with
:math:`\varphi_i:U_i\to\mathbb{R}^{n_i}`, then

.. math::

   U \;\coloneqq\; U_1\times\cdots\times U_k \subset \mathcal{M},\qquad
   \varphi \;\coloneqq\; \varphi_1\times\cdots\times\varphi_k

defines a chart on :math:`\mathcal{M}` via

.. math::

   \varphi(x_1,\dots,x_k)
   \;=\;\big(\varphi_1(x_1),\dots,\varphi_k(x_k)\big)\in\mathbb{R}^{n_1+\cdots+n_k}.

Hence,

.. math::

   \dim(\mathcal{M}_1\times\cdots\times\mathcal{M}_k)
   \;=\;\sum_{i=1}^k \dim(\mathcal{M}_i).

The canonical projections :math:`\pi_i:\mathcal{M}\to\mathcal{M}_i`,
:math:`\pi_i(x_1,\dots,x_k)=x_i`, are smooth.

Tangent space: direct sum structure
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

At a point :math:`x=(x_1,\dots,x_k)\in\mathcal{M}`, the tangent space splits as

.. math::

   T_x\mathcal{M}
   \;\cong\;
   T_{x_1}\mathcal{M}_1 \oplus \cdots \oplus T_{x_k}\mathcal{M}_k.

Intuitively, an “infinitesimal motion” on the product is just a tuple of
infinitesimal motions on each factor. In coordinates, derivatives and Jacobians
often become block-structured, mirroring this direct-sum decomposition.

Riemannian product metric
^^^^^^^^^^^^^^^^^^^^^^^^^

If each :math:`\mathcal{M}_i` has a Riemannian metric :math:`\langle\cdot,\cdot\rangle_i`,
a common choice on the product is

.. math::

   \big\langle (v_1,\dots,v_k),(w_1,\dots,w_k)\big\rangle
   \;\coloneqq\;
   \sum_{i=1}^k \langle v_i,w_i\rangle_i,
   \quad v_i,w_i\in T_{x_i}\mathcal{M}_i.

Lie-group special case
^^^^^^^^^^^^^^^^^^^^^^

If each :math:`\mathcal{M}_i` is a Lie group :math:`G_i`, then the product
:math:`G=G_1\times\cdots\times G_k` is a Lie group with component-wise multiplication:

.. math::

   (g_1,\dots,g_k)\cdot(h_1,\dots,h_k)
   \;=\;
   (g_1h_1,\dots,g_kh_k).

Its Lie algebra is the direct sum

.. math::

   \mathfrak{g}
   \;=\;
   \mathfrak{g}_1 \oplus \cdots \oplus \mathfrak{g}_k,

and exponential/log maps act componentwise:

.. math::

   \exp_G(\xi_1,\dots,\xi_k)=(\exp_{G_1}\xi_1,\dots,\exp_{G_k}\xi_k),\qquad
   \log_G(g_1,\dots,g_k)=(\log_{G_1}g_1,\dots,\log_{G_k}g_k).

In particular, since :math:`\mathbb{R}^n` is a Lie group under addition, the common
state space

.. math::

   SO(3)\times \mathbb{R}^n

is (also) a Lie group under :math:`(R,a)\cdot(S,b)=(RS,a+b)`, with Lie algebra
:math:`\mathfrak{so}(3)\oplus\mathbb{R}^n`.

Engineering view: “mixed” state vectors in flight dynamics
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Many GNC/flight-sim states are naturally tuples of heterogeneous components, e.g.

.. math::

   x \;=\; (R,\;p,\;v,\;b_g,\;b_a,\;\dots)
   \in SO(3)\times \mathbb{R}^3\times \mathbb{R}^3\times \mathbb{R}^3\times \mathbb{R}^3\times\cdots.

The product-manifold viewpoint says:

- You should update each component using the correct geometry.
- Euclidean components update by addition; rotational components update by
  group composition (or a retraction).

A convenient way to formalize this is with “box-plus / box-minus” operators.
Let :math:`\oplus_i` be the update rule for :math:`\mathcal{M}_i` (e.g. :math:`+` on
:math:`\mathbb{R}^n`, left-multiplication by :math:`\exp(\cdot)` on :math:`SO(3)`).
Then define for the product:

.. math::

   (x_1,\dots,x_k)\;\oplus\;(\delta_1,\dots,\delta_k)
   \;\coloneqq\;
   (x_1\oplus_1\delta_1,\;\dots,\;x_k\oplus_k\delta_k),

and similarly for :math:`\ominus` component-wise.

This is exactly what error-state EKFs and Lie-group / RKMK integrators exploit:
propagate on the product manifold, and linearize in the (direct-sum) tangent space.

Why “separate” :math:`\mathbb{R}^n` even though it is a Lie group?
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

You *can* treat everything as a Lie group if you want (e.g. :math:`SO(3)\times\mathbb{R}^n`).
In practice we keep :math:`\mathbb{R}^n` as “plain Euclidean” because:

- its group operation is already :math:`+`, so the manifold machinery adds no
  complexity where it is unnecessary;
- linearization and covariance bookkeeping are simplest on Euclidean blocks;
- non-Euclidean parts (e.g. :math:`SO(3)`, :math:`SE(3)`) are where constraints and
  curvature matter (e.g. quaternion normalization, avoiding singular charts).

The product-manifold framing cleanly supports both: treat curved components with
proper group/retraction updates, and treat Euclidean ones with standard addition.
