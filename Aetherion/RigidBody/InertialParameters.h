// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------

#pragma once

namespace Aetherion::RigidBody{

/// @brief Physical inertial properties of a rigid vehicle.
///
/// All quantities are referenced to the **centre of gravity (CoG)** and
/// expressed in the **body frame** unless noted otherwise.
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

        /// @name Moments of inertia about CoG in body frame [kg·m²]
        ///@{
        double Ixx{ 0.0 }; ///< Roll-axis moment of inertia [kg·m²].
        double Iyy{ 0.0 }; ///< Pitch-axis moment of inertia [kg·m²].
        double Izz{ 0.0 }; ///< Yaw-axis moment of inertia [kg·m²].
        ///@}

        /// @name Products of inertia about CoG in body frame [kg·m²]
        ///@{
        double Ixy{ 0.0 }; ///< xy product of inertia [kg·m²].
        double Iyz{ 0.0 }; ///< yz product of inertia [kg·m²].
        double Ixz{ 0.0 }; ///< xz product of inertia [kg·m²].
        ///@}

        /// @name Body-frame origin offset from CoG [m]
        /// Position of the body-axes origin relative to CoG, expressed in body frame.
        ///@{
        double xbar_m{ 0.0 }; ///< x-offset of body origin from CoG [m].
        double ybar_m{ 0.0 }; ///< y-offset of body origin from CoG [m].
        double zbar_m{ 0.0 }; ///< z-offset of body origin from CoG [m].
        ///@}
    };

} // namespace Aetherion::RigidBosy::Parameters
