// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// Blocks.h
//
// Block library of the estimator error state (sensor fusion step 2).
//
// The error state is composed from named blocks of fixed dimension, units,
// frame and process model. A vehicle configuration (Configuration.h) selects
// blocks at build time; their position in the error vector is fixed by the
// canonical rank below, never by the order a configuration lists them, so two
// configurations that share a block agree on its layout relative to the
// blocks before it.
//
// Filter form: right-invariant EKF on SE2(3) x R^n, navigation frame
// launch-centred inertial (LCI): origin at the launch point, axes equal to
// local NED at t0 and then frozen in inertial space.
//
// The navigation core is one SE2(3) element X = (R, v, p), R body -> LCI,
// v and p in LCI. Its 9 error states are the right-invariant tangent vector
// xi = (phi, nu, rho) of eta = X * Xhat^-1, in that order. nu and rho carry
// units of velocity and position but are not plain differences of v and p:
// v - vhat = nu + [phi]x vhat to first order (likewise rho). Bias blocks are
// Euclidean and additive.
// ------------------------------------------------------------------------------

#pragma once

#include <array>
#include <cstdint>
#include <string_view>

namespace Aetherion::Estimation
{

enum class BlockId : std::uint8_t
{
  NavCore,    ///< SE2(3) attitude, velocity, position
  GyroBias,   ///< Gyroscope bias, body frame
  AccelBias,  ///< Accelerometer bias, body frame
  BaroBias,   ///< Barometer bias in metres of pressure altitude
};

inline constexpr std::size_t kBlockCount = 4;

enum class ProcessModelKind : std::uint8_t
{
  Strapdown,   ///< IMU-driven SE2(3) kinematics in LCI
  RandomWalk,  ///< x_dot = w, w white
};

struct BlockSpec
{
  BlockId id;
  std::string_view name;   ///< Stable identifier; part of the configuration hash
  int dim;                 ///< Error-state dimension
  std::string_view units;  ///< Per-component units, comma-separated groups
  std::string_view frame;  ///< Frame the components are resolved in
  ProcessModelKind process;
  int rank;  ///< Canonical position; lower comes first
};

// Canonical block library. Rank order is the error-vector order; the
// navigation core is always first. New blocks get a new rank and a new
// name; an existing entry is never edited in place (it would silently
// change every configuration hash that uses it -- which is the intent of
// the hash, but should be a deliberate act, not a side effect).
inline constexpr std::array<BlockSpec, kBlockCount> kBlockLibrary{ {
    { BlockId::NavCore, "nav_core", 9, "rad[3],m/s[3],m[3]", "LCI", ProcessModelKind::Strapdown, 0 },
    { BlockId::GyroBias, "gyro_bias", 3, "rad/s[3]", "body", ProcessModelKind::RandomWalk, 1 },
    { BlockId::AccelBias, "accel_bias", 3, "m/s^2[3]", "body", ProcessModelKind::RandomWalk, 2 },
    { BlockId::BaroBias, "baro_bias", 1, "m[1]", "none", ProcessModelKind::RandomWalk, 3 },
} };

[[nodiscard]] constexpr const BlockSpec& blockSpec(BlockId id) { return kBlockLibrary[static_cast<std::size_t>(id)]; }

[[nodiscard]] constexpr std::string_view toString(ProcessModelKind k)
{
  switch (k)
  {
    case ProcessModelKind::Strapdown:
      return "strapdown";
    case ProcessModelKind::RandomWalk:
      return "random_walk";
  }
  return "?";
}

// Offsets inside the navigation core (right-invariant tangent order).
namespace NavCoreLayout
{
inline constexpr int kAttitude = 0;  ///< phi, rad
inline constexpr int kVelocity = 3;  ///< nu,  m/s
inline constexpr int kPosition = 6;  ///< rho, m
}  // namespace NavCoreLayout

// The library must be indexed by BlockId and listed in rank order; the
// offset computation in Configuration.h relies on both.
static_assert(
    [] {
      for (std::size_t i = 0; i < kBlockCount; ++i)
      {
        if (static_cast<std::size_t>(kBlockLibrary[i].id) != i)
          return false;
        if (kBlockLibrary[i].rank != static_cast<int>(i))
          return false;
      }
      return kBlockLibrary[0].id == BlockId::NavCore;
    }(),
    "kBlockLibrary must be indexed by BlockId, in rank order, nav_core first");

}  // namespace Aetherion::Estimation
