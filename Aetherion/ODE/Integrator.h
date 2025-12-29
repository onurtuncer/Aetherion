// ------------------------------------------------------------------------------
// Project: Aetherion
// SPDX-License-Identifier: MIT
// ------------------------------------------------------------------------------
//
// Integrator.h (single-step)
// Bridges:
//   - Flat packed state (Eigen::VectorXd) using StateLayout
//   - Geometric manifold state (SE3 × R6 × R)
//   - Single RKMK step
//
// You provide:
//   - StateLayout: IDX_* and N
//   - GeomState type (SE3, nu, m)
//   - pack/unpack between flat and geom
//   - Stepper: step(t, geom_state, h, system) -> StepResult<GeomState> or GeomState
//

#pragma once

#include <cmath>
#include <functional>
#include <stdexcept>
#include <type_traits>
#include <utility>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace Aetherion::ODE {

    inline bool is_finite(double x) noexcept { return std::isfinite(x); }

    // ---------------------------------------------
    // Default StepResult
    // ---------------------------------------------
    template <class State>
    struct StepResult {
        double t_next{};
        State  x_next{};
        bool   ok{ true };
    };

    // ---------------------------------------------
    // Observer hook (optional)
    // ---------------------------------------------
    using FlatObserver = std::function<void(double t, const Eigen::VectorXd& x_flat)>;

    // ---------------------------------------------
    // Geometric state used by your RKMK integrator
    // ---------------------------------------------
    template <class SE3Type>
    struct GeomState {
        SE3Type gWB;                               // pose in SE(3): (RWB, pW)
        Eigen::Matrix<double, 6, 1> nuB;             // twist in body: [omegaB; vB]
        double m{ 0.0 };                             // mass
    };

    // ---------------------------------------------
    // Adapter concept: pack/unpack between flat and geom
    // ---------------------------------------------
    template <class StateLayout, class SE3Type>
    struct StateAdapter {
        using geom_state_type = GeomState<SE3Type>;

        // --- Unpack flat vector into geometric state
        // Expected flat layout (example):
        //   pW: IDX_P..IDX_P+2
        //   qWB(w,x,y,z): IDX_Q..IDX_Q+3
        //   omegaB: IDX_W..IDX_W+2
        //   vB: IDX_V..IDX_V+2
        //   m: IDX_M
        static geom_state_type unpack(const Eigen::VectorXd& x) {
            if (x.size() != StateLayout::N) {
                throw std::invalid_argument("StateAdapter::unpack: wrong state dimension.");
            }

            geom_state_type gs{};

            const Eigen::Vector3d pW = x.segment<3>(StateLayout::IDX_P);

            Eigen::Quaterniond qWB(
                x(StateLayout::IDX_Q + 0),
                x(StateLayout::IDX_Q + 1),
                x(StateLayout::IDX_Q + 2),
                x(StateLayout::IDX_Q + 3)
            );
			qWB.normalize(); // TODO verify if needed

            // Construct SE3 from (q, p). Your SE3 type may differ; adapt here.
            gs.gWB = SE3Type(qWB, pW);

            const Eigen::Vector3d wB = x.segment<3>(StateLayout::IDX_W);
            const Eigen::Vector3d vB = x.segment<3>(StateLayout::IDX_V);
            gs.nuB << wB, vB;

            gs.m = x(StateLayout::IDX_M);
            return gs;
        }

        // --- Pack geometric state back into flat vector
        static Eigen::VectorXd pack(const geom_state_type& gs) {
            Eigen::VectorXd x(StateLayout::N);
            x.setZero();

            // Extract from SE3. Your API may differ; adapt these accessors.
            const Eigen::Vector3d pW = gs.gWB.translation();
            const Eigen::Quaterniond qWB = gs.gWB.quaternion().normalized();

            x.segment<3>(StateLayout::IDX_P) = pW;

            x(StateLayout::IDX_Q + 0) = qWB.w();
            x(StateLayout::IDX_Q + 1) = qWB.x();
            x(StateLayout::IDX_Q + 2) = qWB.y();
            x(StateLayout::IDX_Q + 3) = qWB.z();

            x.segment<3>(StateLayout::IDX_W) = gs.nuB.head<3>();
            x.segment<3>(StateLayout::IDX_V) = gs.nuB.tail<3>();

            x(StateLayout::IDX_M) = gs.m;
            return x;
        }
    };

    // ---------------------------------------------
    // Single-step Integrator operating on flat state, stepping on geom state.
    // ---------------------------------------------
    template <class StateLayout, class SE3Type, class System, class Stepper>
    class Integrator final {
    public:
        using geom_state_type = GeomState<SE3Type>;
        using adapter_type = StateAdapter<StateLayout, SE3Type>;
        using result_geom = StepResult<geom_state_type>;
        using result_flat = StepResult<Eigen::VectorXd>;

        explicit Integrator(Stepper stepper) : stepper_(std::move(stepper)) {}

        Stepper& stepper()       noexcept { return stepper_; }
        const Stepper& stepper() const noexcept { return stepper_; }

        // Flat in, flat out (one step).
        result_flat step(double t,
            const Eigen::VectorXd& x_flat,
            double h,
            const System& sys,
            FlatObserver obs = nullptr) const
        {
            validate_inputs(t, h);

            const geom_state_type xg = adapter_type::unpack(x_flat);

            // Stepper may return StepResult<geom_state_type> or geom_state_type
            geom_state_type xg_next{};
            double t_next = t + h;

            if constexpr (requires(const Stepper & st, double tt, const geom_state_type & xx, double hh, const System & ss) {
                { st.step(tt, xx, hh, ss) } -> std::same_as<result_geom>;
            })
            {
                auto r = stepper_.step(t, xg, h, sys);
                if (!r.ok) throw std::runtime_error("Integrator: stepper reported failure.");
                xg_next = std::move(r.x_next);
                t_next = r.t_next;
            }
            else if constexpr (requires(const Stepper & st, double tt, const geom_state_type & xx, double hh, const System & ss) {
                { st.step(tt, xx, hh, ss) } -> std::same_as<geom_state_type>;
            })
            {
                xg_next = stepper_.step(t, xg, h, sys);
            }
            else {
                static_assert(sizeof(Stepper) == 0,
                    "Stepper must implement step(t, geom_state, h, sys) returning StepResult<GeomState> or GeomState.");
            }

            Eigen::VectorXd x_flat_next = adapter_type::pack(xg_next);
            if (obs) obs(t_next, x_flat_next);

            return result_flat{ t_next, std::move(x_flat_next), true };
        }

    private:
        static void validate_inputs(double t, double h) {
            if (!is_finite(t) || !is_finite(h)) {
                throw std::invalid_argument("Integrator: t and h must be finite.");
            }
            if (h <= 0.0) {
                throw std::invalid_argument("Integrator: step size h must be > 0.");
            }
        }

    private:
        mutable Stepper stepper_;
    };

} // namespace Aetherion::Int
