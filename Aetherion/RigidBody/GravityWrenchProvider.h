// Aetherion/RigidBody/Providers/GravityWrenchProvider.h
#pragma once

#include <Eigen/Dense>

#include "Aetherion/RigidBody/IWrenchProvider.h"
#include "Aetherion/Spatial/Wrench.h"
#include "Aetherion/RigidBody/GravitationalWrench.h" // your file

namespace Aetherion::RigidBody {

    template<class Scalar>
    class GravityWrenchProvider final : public IWrenchProvider<Scalar>
    {
    public:
        enum class Model { Central, J2 };

        explicit GravityWrenchProvider(Model model = Model::Central)
            : m_Model(model) {
        }

        Aetherion::Spatial::Wrench<Scalar>
            ComputeWrench_B_at_CG(const WrenchContext<Scalar>& ctx) const override
        {
            // 1) Compute gravity wrench in inertial frame W (at CG)
            Aetherion::Spatial::Wrench<Scalar> w_W{};

            if (m_Model == Model::Central)
            {
                w_W = Aetherion::RigidBody::GravitationalWrenchAtCG(
                    ctx.r_W, ctx.mass_kg, ctx.mu_m3_s2);
            }
            else
            {
                w_W = Aetherion::RigidBody::GravitationalWrenchJ2AtCG(
                    ctx.r_W, ctx.mass_kg, ctx.mu_m3_s2, ctx.Re_m, ctx.J2);
            }

            // 2) Rotate wrench into body frame B
            // Since wrench is at CG, translation does not matter.
            Aetherion::Spatial::Wrench<Scalar> w_B{};
            w_B.f.template segment<3>(0) = ctx.R_BW * w_W.f.template segment<3>(0); // moment
            w_B.f.template segment<3>(3) = ctx.R_BW * w_W.f.template segment<3>(3); // force
            return w_B;
        }

    private:
        Model m_Model;
    };

} // namespace Aetherion::RigidBody