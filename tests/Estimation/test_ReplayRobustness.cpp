// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// Sensor fusion step 6, replay harness: inputs that are not numbers.
//
// Regression for the pre-DOI audit's finding F10. Csv.h turns an empty or
// unparseable cell into NaN. gpsRecords() used to accept such a row as a valid
// fix (position and fix type finite) and, with the velocity columns merely
// present, as one with a velocity; the NaN then went through an update and
// every later estimate was NaN, with no fault reported. Two guards now stand
// in the way: the loader does not call a row with NaN in any field the update
// reads a fix, and the filter refuses an innovation or covariance that is not
// finite, or an S that does not factorise positive.
// ------------------------------------------------------------------------------

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <limits>
#include <numbers>
#include <sstream>
#include <string_view>

#include <Aetherion/Estimation/EKF.h>
#include <Aetherion/Estimation/Replay/Csv.h>
#include <Aetherion/Estimation/Replay/Logs.h>

using namespace Aetherion::Estimation;
using namespace Aetherion::Estimation::Replay;

namespace
{
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kLat0 = 36.0192, kLon0 = -75.6744, kAlt0 = 3000.0;
}  // namespace

TEST_CASE("Logs: GPS rows with NaN cells are not usable fixes", "[estimation][replay][logs][negative][regression]")
{
  constexpr std::string_view header =
      "fix_count,nominal_time_s,host_time_s,fix_type,latitude_deg,longitude_deg,altitude_m,"
      "horizontal_accuracy_m,vertical_accuracy_m,vel_north_mps,vel_east_mps,vel_down_mps,speed_accuracy_mps";
  const Configuration cfg{ { BlockId::NavCore }, { MeasurementId::GnssPosition, MeasurementId::GnssVelocity } };

  std::stringstream in;
  in << header << "\n"
     << "0,0.0,0.1,2,36.0192,-75.6744,3000,1.5,3.0,1,2,3,0.1\n"        // complete
     << "1,0.2,0.3,2,36.0192,-75.6744,3000,1.5,3.0,,2,3,0.1\n"         // velocity cell empty
     << "2,0.4,0.5,2,36.0192,-75.6744,3000,1.5,3.0,1,2,3,\n"           // speed accuracy empty
     << "3,0.6,0.7,2,36.0192,-75.6744,3000,,3.0,1,2,3,0.1\n"           // horizontal accuracy empty
     << "4,0.8,0.9,2,36.0192,-75.6744,3000,1.5,nan,1,2,3,0.1\n"        // vertical accuracy NaN
     << "5,1.0,1.1,,,,,,,,,,\n";                                        // epoch without a fix
  const auto gps = gpsRecords(readCsv(in, LogContract::Log::GpsFixes, cfg));
  REQUIRE(gps.size() == 6);

  CHECK(gps[0].valid);
  CHECK(gps[0].hasVelocity);

  CHECK(gps[1].valid);           // the position is still a fix
  CHECK_FALSE(gps[1].hasVelocity);
  CHECK(gps[2].valid);
  CHECK_FALSE(gps[2].hasVelocity);

  CHECK_FALSE(gps[3].valid);     // nothing of this row may be used
  CHECK_FALSE(gps[3].hasVelocity);
  CHECK_FALSE(gps[4].valid);
  CHECK_FALSE(gps[4].hasVelocity);

  CHECK_FALSE(gps[5].valid);
  CHECK_FALSE(gps[5].hasVelocity);
}

TEST_CASE("EKF: a measurement that is not a number is rejected and changes nothing", "[estimation][ekf][regression]")
{
  constexpr double kDeg = std::numbers::pi / 180.0;
  const Configuration cfg{ { BlockId::NavCore }, { MeasurementId::GnssPosition, MeasurementId::GnssVelocity } };
  const LciFrame<double> frame = LciFrame<double>::FromGeodetic(kLat0 * kDeg, kLon0 * kDeg, kAlt0, 0.0);
  InvariantEKF<double, ErrorConvention::RightInvariant> ekf(cfg, frame, ProcessNoise<double>{ 2e-4, 5e-3, 2e-6, 2e-5, 0.05 });

  const auto x0 = ekf.state();
  const Eigen::MatrixXd P0 = ekf.covariance();
  const Eigen::Matrix3d Sigma = Eigen::Matrix3d::Identity();

  const auto v = ekf.updateGnssVelocity(Eigen::Vector3d(0.0, kNaN, 0.0), Sigma);
  CHECK_FALSE(v.applied);
  CHECK(v.rejected);
  CHECK(v.dim == 3);

  const auto p = ekf.updateGnssPosition(Eigen::Vector3d::Zero(), Eigen::Matrix3d::Constant(kNaN));
  CHECK_FALSE(p.applied);
  CHECK(p.rejected);

  // An innovation covariance that is not positive definite is refused too.
  const auto q = ekf.updateGnssPosition(Eigen::Vector3d::Zero(), -1e6 * Eigen::Matrix3d::Identity());
  CHECK_FALSE(q.applied);
  CHECK(q.rejected);

  CHECK(ekf.state().X.p == x0.X.p);
  CHECK(ekf.state().X.v == x0.X.v);
  CHECK(ekf.state().X.R == x0.X.R);
  CHECK(ekf.covariance() == P0);
  CHECK(ekf.covariance().allFinite());

  // A good measurement afterwards is applied as usual.
  const auto ok = ekf.updateGnssPosition(Eigen::Vector3d(1.0, 0.0, 0.0), Sigma);
  CHECK(ok.applied);
  CHECK_FALSE(ok.rejected);
  CHECK(std::isfinite(ok.nis));
  CHECK(ekf.state().X.p.allFinite());
}
