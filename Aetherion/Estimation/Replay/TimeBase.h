// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// TimeBase.h
//
// The time-base calibration of the replay harness (sensor fusion step 6;
// doc/sensor_fusion.rst "Replay and log contract"). Hemerion's
// `host_time_s` is the flight computer's clock and equals simulation time
// only under a real-time factor of 1; the truth log's `time` is simulation
// time; the IMU log carries both `host_time_s` and the sensor's own
// `part_time_s`. The harness fits
//
//     host_time = offset + scale * part_time
//
// by least squares over the IMU log and maps every host stamp onto the
// part clock with the inverse. Whether the part clock *is* simulation time
// is Hemerion's to say (the IMU FMU models clock skew and jitter as options);
// the fit's residual statistics are reported so a bad assumption shows up
// as a residual, not as a silent offset. The LCI frame needs the mapping to
// be good to well under 1 ms: a timing error dt rotates the ECEF -> LCI
// mapping by w_e dt, about 0.47 m per millisecond at the Earth's radius.
// ------------------------------------------------------------------------------

#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <vector>

namespace Aetherion::Estimation::Replay
{

struct TimeBaseFit
{
  double offset = 0.0;  ///< host = offset + scale * part
  double scale = 1.0;
  double rmsResidual = 0.0;  ///< of host - (offset + scale * part), seconds
  double maxResidual = 0.0;
  std::size_t samples = 0;

  /// Part-clock (nominally simulation) time of a host stamp.
  [[nodiscard]] double toPart(double host) const { return (host - offset) / scale; }
  [[nodiscard]] double toHost(double part) const { return offset + scale * part; }
};

/// Least-squares line through (part, host) pairs. Two or more distinct part
/// times are required.
[[nodiscard]] inline TimeBaseFit fitTimeBase(const std::vector<double>& part, const std::vector<double>& host)
{
  if (part.size() != host.size())
    throw std::invalid_argument("fitTimeBase: part and host differ in length");
  const std::size_t n = part.size();
  if (n < 2)
    throw std::invalid_argument("fitTimeBase: fewer than two samples");

  // Centred for conditioning: part times are seconds since start, host
  // times may be a wall clock.
  double mp = 0.0, mh = 0.0;
  for (std::size_t i = 0; i < n; ++i)
  {
    mp += part[i];
    mh += host[i];
  }
  mp /= static_cast<double>(n);
  mh /= static_cast<double>(n);
  double spp = 0.0, sph = 0.0;
  for (std::size_t i = 0; i < n; ++i)
  {
    const double dp = part[i] - mp;
    spp += dp * dp;
    sph += dp * (host[i] - mh);
  }
  if (spp <= 0.0)
    throw std::invalid_argument("fitTimeBase: all part times equal");

  TimeBaseFit f;
  f.scale = sph / spp;
  f.offset = mh - f.scale * mp;
  f.samples = n;
  double ss = 0.0;
  for (std::size_t i = 0; i < n; ++i)
  {
    const double r = host[i] - f.toHost(part[i]);
    ss += r * r;
    f.maxResidual = std::max(f.maxResidual, std::abs(r));
  }
  f.rmsResidual = std::sqrt(ss / static_cast<double>(n));
  return f;
}

}  // namespace Aetherion::Estimation::Replay
