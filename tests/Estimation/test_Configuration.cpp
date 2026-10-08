// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------

#include <catch2/catch_test_macros.hpp>

#include <Aetherion/Estimation/Configuration.h>

using namespace Aetherion::Estimation;

TEST_CASE("First configuration: 16 states in canonical order", "[estimation][configuration]")
{
  const Configuration& c = kNavBaroGnss16;
  REQUIRE(c.valid());
  REQUIRE(c.dimension() == 16);
  REQUIRE(c.offset(BlockId::NavCore) == 0);
  REQUIRE(c.offset(BlockId::GyroBias) == 9);
  REQUIRE(c.offset(BlockId::AccelBias) == 12);
  REQUIRE(c.offset(BlockId::BaroBias) == 15);
}

TEST_CASE("Block order is canonical, not insertion order", "[estimation][configuration]")
{
  const Configuration a{ { BlockId::BaroBias, BlockId::GyroBias, BlockId::NavCore }, {} };
  const Configuration b{ { BlockId::NavCore, BlockId::GyroBias, BlockId::BaroBias }, {} };
  REQUIRE(a.offset(BlockId::GyroBias) == 9);
  REQUIRE(a.offset(BlockId::BaroBias) == 12);  // accel bias absent: no gap
  REQUIRE(a.offset(BlockId::AccelBias) == -1);
  REQUIRE(a.hash() == b.hash());
}

TEST_CASE("A measurement is valid only with its blocks", "[estimation][configuration]")
{
  const Configuration noBaroBias{ { BlockId::NavCore }, { MeasurementId::Barometer } };
  REQUIRE_FALSE(noBaroBias.valid());

  const Configuration noCore{ { BlockId::BaroBias }, {} };
  REQUIRE_FALSE(noCore.valid());

  const Configuration gnssOnly{ { BlockId::NavCore }, { MeasurementId::GnssPosition } };
  REQUIRE(gnssOnly.valid());
}

TEST_CASE("Hash distinguishes block and measurement sets", "[estimation][configuration]")
{
  const Configuration withoutBaro{ { BlockId::NavCore, BlockId::GyroBias, BlockId::AccelBias },
                                   { MeasurementId::GnssPosition, MeasurementId::GnssVelocity } };
  const Configuration withoutVel{ { BlockId::NavCore, BlockId::GyroBias, BlockId::AccelBias, BlockId::BaroBias },
                                  { MeasurementId::GnssPosition, MeasurementId::Barometer } };
  REQUIRE(withoutBaro.hash() != kNavBaroGnss16.hash());
  REQUIRE(withoutVel.hash() != kNavBaroGnss16.hash());
}

// Pinned on purpose. If this fails, the first configuration's descriptor has
// changed, so every generated file and golden vector carrying the old hash is
// stale. Update the pin only together with regenerating them.
TEST_CASE("First configuration descriptor and hash are pinned", "[estimation][configuration]")
{
  REQUIRE(kNavBaroGnss16.descriptor() == "aetherion.estimation/1\n"
                                         "group=SE2(3)\n"
                                         "error=right-invariant\n"
                                         "frame=LCI\n"
                                         "block=nav_core,9,rad[3],m/s[3],m[3],LCI,strapdown\n"
                                         "block=gyro_bias,3,rad/s[3],body,random_walk\n"
                                         "block=accel_bias,3,m/s^2[3],body,random_walk\n"
                                         "block=baro_bias,1,m[1],none,random_walk\n"
                                         "meas=gnss_position,3,nav_core\n"
                                         "meas=gnss_velocity,3,nav_core\n"
                                         "meas=barometer,1,nav_core+baro_bias\n");
  REQUIRE(kNavBaroGnss16.hash() == 0x1e59a990edd19e83ull);
}
