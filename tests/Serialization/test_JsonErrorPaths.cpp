// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// The optional sections of the scenario JSON (wind, windShear) and the
// diagnostics every reader produces for a missing or mistyped key: the
// message must name the section and the key, since it is all a user sees
// of a malformed scenario file.
// ------------------------------------------------------------------------------

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <vendor/nlohmann/json.hpp>

#include "Aetherion/RigidBody/Config.h"
#include "Aetherion/Serialization/Json/AerodynamicParameters.h"
#include "Aetherion/Serialization/Json/BodyRates.h"
#include "Aetherion/Serialization/Json/Config.h"
#include "Aetherion/Serialization/Json/GeodeticPoseNED.h"
#include "Aetherion/Serialization/Json/InertialParameters.h"
#include "Aetherion/Serialization/Json/VelocityNED.h"

namespace Ser = ::Aetherion::Serialization::Json;
namespace RigidBody = ::Aetherion::RigidBody;
using Catch::Matchers::ContainsSubstring;

namespace
{
nlohmann::json validConfig()
{
  return nlohmann::json::parse(R"({
    "pose": { "lat_deg": 41.1, "lon_deg": 29.0, "alt_m": 100.0,
              "azimuth_deg": 90.0, "zenith_deg": 5.0, "roll_deg": 0.0 },
    "velocityNED": { "north_mps": 1.0, "east_mps": 2.0, "down_mps": 3.0 },
    "bodyRates":   { "roll_rad_s": 0.1, "pitch_rad_s": 0.2, "yaw_rad_s": 0.3 },
    "inertialParameters": {
      "mass_kg": 10.0,
      "inertia_kgm2": { "Ixx": 0.1, "Iyy": 0.2, "Izz": 0.3, "Ixy": 0.0, "Iyz": 0.0, "Ixz": 0.0 },
      "body_origin_wrt_cog_m": { "x": 0.0, "y": 0.0, "z": 0.0 }
    },
    "aerodynamicParameters": { "S": 0.1, "CL": 0.0, "CD": 0.0, "CY": 0.0, "Cl": 0.0, "Cm": 0.0, "Cn": 0.0 }
  })");
}
}  // namespace

TEST_CASE("Config JSON: wind and wind shear are optional and default to calm", "[json][config][wind]")
{
  RigidBody::Config cfg{};
  Ser::from_json(validConfig(), cfg);
  CHECK(cfg.wind.north_mps == 0.0);
  CHECK(cfg.wind.east_mps == 0.0);
  CHECK(cfg.wind.down_mps == 0.0);
  CHECK(cfg.windShear.gradient_N_mps_m == 0.0);
  CHECK(cfg.windShear.intercept_E_mps == 0.0);
}

TEST_CASE("Config JSON: wind and wind shear sections are read", "[json][config][wind]")
{
  nlohmann::json j = validConfig();
  j["wind"] = { { "north_mps", 5.0 }, { "east_mps", -3.0 }, { "down_mps", 0.5 } };
  j["windShear"] = { { "gradient_N_mps_m", 0.01 },
                     { "gradient_E_mps_m", -0.02 },
                     { "intercept_N_mps", 1.5 },
                     { "intercept_E_mps", -2.5 } };

  RigidBody::Config cfg{};
  Ser::from_json(j, cfg);
  CHECK(cfg.wind.north_mps == Catch::Approx(5.0));
  CHECK(cfg.wind.east_mps == Catch::Approx(-3.0));
  CHECK(cfg.wind.down_mps == Catch::Approx(0.5));
  CHECK(cfg.windShear.gradient_N_mps_m == Catch::Approx(0.01));
  CHECK(cfg.windShear.gradient_E_mps_m == Catch::Approx(-0.02));
  CHECK(cfg.windShear.intercept_N_mps == Catch::Approx(1.5));
  CHECK(cfg.windShear.intercept_E_mps == Catch::Approx(-2.5));

  // The other sections are untouched by the optional ones.
  CHECK(cfg.velocityNED.east_mps == Catch::Approx(2.0));
  CHECK(cfg.bodyRates.yaw_rad_s == Catch::Approx(0.3));
}

TEST_CASE("Config JSON: a partial wind section keeps the defaults of the absent keys", "[json][config][wind]")
{
  nlohmann::json j = validConfig();
  j["wind"] = { { "east_mps", 7.0 } };
  j["windShear"] = { { "gradient_E_mps_m", 0.004 } };

  RigidBody::Config cfg{};
  Ser::from_json(j, cfg);
  CHECK(cfg.wind.north_mps == 0.0);
  CHECK(cfg.wind.east_mps == Catch::Approx(7.0));
  CHECK(cfg.wind.down_mps == 0.0);
  CHECK(cfg.windShear.gradient_N_mps_m == 0.0);
  CHECK(cfg.windShear.gradient_E_mps_m == Catch::Approx(0.004));
  CHECK(cfg.windShear.intercept_N_mps == 0.0);
  CHECK(cfg.windShear.intercept_E_mps == 0.0);
}

TEST_CASE("Config JSON: errors name the section and the key", "[json][config][negative]")
{
  RigidBody::Config cfg{};

  SECTION("missing top-level section")
  {
    for (const char* section : { "pose", "velocityNED", "bodyRates", "inertialParameters", "aerodynamicParameters" })
    {
      nlohmann::json j = validConfig();
      j.erase(section);
      INFO(section);
      REQUIRE_THROWS_WITH(Ser::from_json(j, cfg),
                          ContainsSubstring(std::string("Config[") + section + "]") &&
                              ContainsSubstring("missing top-level key"));
    }
  }

  SECTION("error inside a section is wrapped with the section name")
  {
    nlohmann::json j = validConfig();
    j["velocityNED"].erase("east_mps");
    REQUIRE_THROWS_WITH(Ser::from_json(j, cfg),
                        ContainsSubstring("Config[velocityNED]") && ContainsSubstring("east_mps"));
  }
}

TEST_CASE("Config JSON: to_json then from_json round-trips through the wrapper", "[json][config]")
{
  RigidBody::Config in{};
  Ser::from_json(validConfig(), in);
  nlohmann::json j;
  Ser::to_json(j, in);
  RigidBody::Config out{};
  Ser::from_json(j, out);
  CHECK(out.pose.azimuth_deg == Catch::Approx(90.0));
  CHECK(out.velocityNED.down_mps == Catch::Approx(3.0));
  CHECK(out.inertialParameters.Izz == Catch::Approx(0.3));
}

TEST_CASE("InertialParameters JSON: each sub-object reports its own failure", "[json][inertial][negative]")
{
  const nlohmann::json good = validConfig()["inertialParameters"];
  RigidBody::InertialParameters ip{};
  REQUIRE_NOTHROW(Ser::from_json(good, ip));

  SECTION("mass")
  {
    nlohmann::json j = good;
    j.erase("mass_kg");
    REQUIRE_THROWS_WITH(Ser::from_json(j, ip), ContainsSubstring("inertialParameters.mass_kg"));
  }
  SECTION("inertia tensor")
  {
    nlohmann::json j = good;
    j["inertia_kgm2"]["Iyz"] = "zero";
    REQUIRE_THROWS_WITH(Ser::from_json(j, ip), ContainsSubstring("inertialParameters.inertia_kgm2"));
  }
  SECTION("body origin")
  {
    nlohmann::json j = good;
    j["body_origin_wrt_cog_m"].erase("z");
    REQUIRE_THROWS_WITH(Ser::from_json(j, ip), ContainsSubstring("inertialParameters.body_origin_wrt_cog_m"));
  }
}

TEST_CASE("GeodeticPoseNED JSON: missing and non-numeric keys", "[json][initial_pose][negative]")
{
  const nlohmann::json good = validConfig()["pose"];
  RigidBody::GeodeticPoseNED pose{};

  nlohmann::json missing = good;
  missing.erase("zenith_deg");
  REQUIRE_THROWS_WITH(Ser::from_json(missing, pose),
                      ContainsSubstring("missing key") && ContainsSubstring("zenith_deg"));

  nlohmann::json wrongType = good;
  wrongType["alt_m"] = "high";
  REQUIRE_THROWS_WITH(Ser::from_json(wrongType, pose),
                      ContainsSubstring("not a number") && ContainsSubstring("alt_m"));
}

TEST_CASE("VelocityNED and BodyRates JSON: non-numeric keys", "[json][negative]")
{
  nlohmann::json v = validConfig()["velocityNED"];
  v["down_mps"] = nullptr;
  RigidBody::VelocityNED vel{};
  REQUIRE_THROWS_WITH(Ser::from_json(v, vel), ContainsSubstring("not a number") && ContainsSubstring("down_mps"));

  nlohmann::json w = validConfig()["bodyRates"];
  w["roll_rad_s"] = nlohmann::json::array({ 1, 2 });
  RigidBody::BodyRates rates{};
  REQUIRE_THROWS_WITH(Ser::from_json(w, rates), ContainsSubstring("not a number") && ContainsSubstring("roll_rad_s"));

  nlohmann::json m = validConfig()["velocityNED"];
  m.erase("north_mps");
  REQUIRE_THROWS_WITH(Ser::from_json(m, vel), ContainsSubstring("missing key") && ContainsSubstring("north_mps"));
}

TEST_CASE("AerodynamicParameters JSON: optional derivatives are read and type-checked", "[json][aero]")
{
  nlohmann::json j = validConfig()["aerodynamicParameters"];
  RigidBody::AerodynamicParameters ap{};

  j["b"] = 9.14;
  j["cbar"] = 3.45;
  j["Cmq"] = -8.0;
  Ser::from_json(j, ap);
  CHECK(ap.b == Catch::Approx(9.14));
  CHECK(ap.cbar == Catch::Approx(3.45));
  CHECK(ap.Cmq == Catch::Approx(-8.0));
  CHECK(ap.Clp == 0.0);  // absent: struct default

  j["Cnr"] = "damped";
  REQUIRE_THROWS_WITH(Ser::from_json(j, ap), ContainsSubstring("not a number") && ContainsSubstring("Cnr"));

  nlohmann::json bad = validConfig()["aerodynamicParameters"];
  bad["CL"] = nlohmann::json::object({ { "value", 0.5 } });
  REQUIRE_THROWS_WITH(Ser::from_json(bad, ap), ContainsSubstring("not a number") && ContainsSubstring("CL"));
}
