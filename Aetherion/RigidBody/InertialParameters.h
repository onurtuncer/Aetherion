// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------

#pragma once

#include <Eigen/Dense>

namespace Aetherion::RigidBody{

/// @brief Physical inertial properties of a rigid vehicle.
///
/// The inertia tensor is about the **centre of mass**, and the offset is the
/// centre of mass **relative to the body-axes origin** (Featherstone's
/// @f$c@f$); everything is expressed in the **body frame**. This is the
/// AIAA S-119 / DAVE-ML convention: @c XIXX .. @c XIYZ are about the centre of
/// mass, and @c DXCG is the centre of mass forward of the moment reference
/// centre, which is the body origin. @c RigidBody::VectorField shifts the
/// tensor to the origin itself (parallel-axis term), so a caller never does.
///
/// The inertia tensor is stored in its standard symmetric form:
/// @f[
///   I = \begin{bmatrix}
///         I_{xx}  & -I_{xy} & -I_{xz} \\
///        -I_{xy}  &  I_{yy} & -I_{yz} \\
///        -I_{xz}  & -I_{yz} &  I_{zz}
///       \end{bmatrix}
/// @f]
/// (products of inertia are positive by convention, stored with leading minus
/// signs in the tensor).
    struct InertialParameters
    {
        double mass_kg{ 0.0 }; ///< Vehicle mass [kg].

        /// @name Moments of inertia about the centre of mass, body frame [kg·m²]
        ///@{
        double Ixx{ 0.0 }; ///< Roll-axis moment of inertia [kg·m²].
        double Iyy{ 0.0 }; ///< Pitch-axis moment of inertia [kg·m²].
        double Izz{ 0.0 }; ///< Yaw-axis moment of inertia [kg·m²].
        ///@}

        /// @name Products of inertia about the centre of mass, body frame [kg·m²]
        ///@{
        double Ixy{ 0.0 }; ///< xy product of inertia [kg·m²].
        double Iyz{ 0.0 }; ///< yz product of inertia [kg·m²].
        double Ixz{ 0.0 }; ///< xz product of inertia [kg·m²].
        ///@}

        /// @name Centre of mass relative to the body-axes origin [m]
        /// Position of the centre of mass with respect to the body origin (the
        /// moment reference centre), in body axes: positive @c xbar_m when the
        /// centre of mass is ahead of the origin (DAVE-ML @c DXCG, sign FWD).
        ///@{
        double xbar_m{ 0.0 }; ///< x of the centre of mass from the body origin [m].
        double ybar_m{ 0.0 }; ///< y of the centre of mass from the body origin [m].
        double zbar_m{ 0.0 }; ///< z of the centre of mass from the body origin [m].
        ///@}
    };

    /// @brief Inertia tensor about the centre of mass in body axes, as a matrix.
    [[nodiscard]] inline Eigen::Matrix3d inertiaTensor(const InertialParameters& ip)
    {
        Eigen::Matrix3d I;
        I <<  ip.Ixx, -ip.Ixy, -ip.Ixz,
             -ip.Ixy,  ip.Iyy, -ip.Iyz,
             -ip.Ixz, -ip.Iyz,  ip.Izz;
        return I;
    }


} // namespace Aetherion::RigidBody
