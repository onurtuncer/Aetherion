// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// Configuration.h
//
// Vehicle configuration of the estimator (sensor fusion step 2): which blocks
// the error state carries, which measurements update it, and a hash of both.
//
// - Blocks are laid out in canonical rank order (Blocks.h), whatever order
//   they were added in.
// - Each measurement declares the blocks it depends on; a configuration that
//   selects a measurement without its blocks is invalid.
// - Measurement availability at run time is handled by skipping updates, never
//   by changing the configuration: the state dimension is fixed at build time.
// - The hash is FNV-1a 64 over a canonical text descriptor that also names the
//   group, the error convention and the navigation frame. Generated code and
//   golden vectors (step 8) carry it, so Hemerion can reject a mismatch.
// ------------------------------------------------------------------------------

#pragma once

#include <array>
#include <cstdint>
#include <initializer_list>
#include <string>
#include <string_view>

#include <Aetherion/Estimation/Blocks.h>

namespace Aetherion::Estimation
{

enum class MeasurementId : std::uint8_t
{
  GnssPosition,  ///< UBX-NAV-PVT lat, lon, height above ellipsoid
  GnssVelocity,  ///< UBX-NAV-PVT velN, velE, velD
  Barometer,     ///< Pressure altitude from compensated pressure, plus bias
};

inline constexpr std::size_t kMeasurementCount = 3;

struct MeasurementSpec
{
  MeasurementId id;
  std::string_view name;  ///< Stable identifier; part of the configuration hash
  int dim;
  std::uint32_t requires_;  ///< Bitmask over BlockId
};

[[nodiscard]] constexpr std::uint32_t bit(BlockId b) { return 1u << static_cast<unsigned>(b); }
[[nodiscard]] constexpr std::uint32_t bit(MeasurementId m) { return 1u << static_cast<unsigned>(m); }

inline constexpr std::array<MeasurementSpec, kMeasurementCount> kMeasurementLibrary{ {
    { MeasurementId::GnssPosition, "gnss_position", 3, bit(BlockId::NavCore) },
    { MeasurementId::GnssVelocity, "gnss_velocity", 3, bit(BlockId::NavCore) },
    { MeasurementId::Barometer, "barometer", 1, bit(BlockId::NavCore) | bit(BlockId::BaroBias) },
} };

[[nodiscard]] constexpr const MeasurementSpec& measurementSpec(MeasurementId id)
{
  return kMeasurementLibrary[static_cast<std::size_t>(id)];
}

static_assert(
    [] {
      for (std::size_t i = 0; i < kMeasurementCount; ++i)
        if (static_cast<std::size_t>(kMeasurementLibrary[i].id) != i)
          return false;
      return true;
    }(),
    "kMeasurementLibrary must be indexed by MeasurementId");

// Fixed parts of the filter form; part of the hash.
inline constexpr std::string_view kDescriptorVersion = "aetherion.estimation/1";
inline constexpr std::string_view kGroup = "SE2(3)";
// Provisional: left- vs right-invariant is still open (doc/sensor_fusion.rst).
// Changing it changes every configuration hash.
inline constexpr std::string_view kErrorConvention = "right-invariant";
inline constexpr std::string_view kNavigationFrame = "LCI";

class Configuration
{
public:
  constexpr Configuration() = default;

  constexpr Configuration(std::initializer_list<BlockId> blocks, std::initializer_list<MeasurementId> measurements)
  {
    for (BlockId b : blocks)
      blocks_ |= bit(b);
    for (MeasurementId m : measurements)
      measurements_ |= bit(m);
  }

  [[nodiscard]] constexpr bool has(BlockId b) const { return (blocks_ & bit(b)) != 0; }
  [[nodiscard]] constexpr bool has(MeasurementId m) const { return (measurements_ & bit(m)) != 0; }

  [[nodiscard]] constexpr std::uint32_t blockMask() const { return blocks_; }
  [[nodiscard]] constexpr std::uint32_t measurementMask() const { return measurements_; }

  /// Error-state dimension.
  [[nodiscard]] constexpr int dimension() const
  {
    int n = 0;
    for (const BlockSpec& s : kBlockLibrary)
      if (has(s.id))
        n += s.dim;
    return n;
  }

  /// Offset of block `b` in the error vector, or -1 if not carried.
  [[nodiscard]] constexpr int offset(BlockId b) const
  {
    if (!has(b))
      return -1;
    int n = 0;
    for (const BlockSpec& s : kBlockLibrary)
    {  // rank order
      if (s.id == b)
        return n;
      if (has(s.id))
        n += s.dim;
    }
    return -1;
  }

  /// True if every selected measurement has its blocks, and the
  /// navigation core is present.
  [[nodiscard]] constexpr bool valid() const
  {
    if (!has(BlockId::NavCore))
      return false;
    for (const MeasurementSpec& m : kMeasurementLibrary)
      if (has(m.id) && (m.requires_ & ~blocks_) != 0)
        return false;
    return true;
  }

  /// Canonical descriptor; the hash input. One line per item, blocks in
  /// rank order, measurements in library order.
  [[nodiscard]] std::string descriptor() const
  {
    std::string d;
    auto line = [&d](std::initializer_list<std::string_view> parts) {
      for (std::string_view p : parts)
        d.append(p);
      d.push_back('\n');
    };
    line({ kDescriptorVersion });
    line({ "group=", kGroup });
    line({ "error=", kErrorConvention });
    line({ "frame=", kNavigationFrame });
    for (const BlockSpec& s : kBlockLibrary)
    {
      if (!has(s.id))
        continue;
      const std::string dim = std::to_string(s.dim);
      line({ "block=", s.name, ",", dim, ",", s.units, ",", s.frame, ",", toString(s.process) });
    }
    for (const MeasurementSpec& m : kMeasurementLibrary)
    {
      if (!has(m.id))
        continue;
      const std::string dim = std::to_string(m.dim);
      std::string deps;
      for (const BlockSpec& s : kBlockLibrary)
      {
        if ((m.requires_ & bit(s.id)) == 0)
          continue;
        if (!deps.empty())
          deps.push_back('+');
        deps.append(s.name);
      }
      line({ "meas=", m.name, ",", dim, ",", deps });
    }
    return d;
  }

  /// FNV-1a 64 of descriptor().
  [[nodiscard]] std::uint64_t hash() const
  {
    std::uint64_t h = 0xcbf29ce484222325ull;
    for (unsigned char c : descriptor())
    {
      h ^= c;
      h *= 0x100000001b3ull;
    }
    return h;
  }

private:
  std::uint32_t blocks_ = 0;
  std::uint32_t measurements_ = 0;
};

/// First configuration (decided 2026-10-08): 16 error states, updated by
/// GNSS position, GNSS velocity and barometer. No magnetometer until
/// Hemerion's replay equivalence (its step 9) has passed. No GNSS antenna
/// lever arm.
inline constexpr Configuration kNavBaroGnss16{
  { BlockId::NavCore, BlockId::GyroBias, BlockId::AccelBias, BlockId::BaroBias },
  { MeasurementId::GnssPosition, MeasurementId::GnssVelocity, MeasurementId::Barometer },
};

static_assert(kNavBaroGnss16.valid());
static_assert(kNavBaroGnss16.dimension() == 16);

}  // namespace Aetherion::Estimation
