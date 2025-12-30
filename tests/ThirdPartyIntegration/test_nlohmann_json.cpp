// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX - License - Identifier: MIT
// License - Filename: LICENSE
// ------------------------------------------------------------------------------

#include <catch2/catch_test_macros.hpp>
#include <nlohmann/json.hpp>

TEST_CASE("nlohmann::json vendored single-header integrates and instantiates", "[vendor][nlohmann][json]") {
    using Json = nlohmann::json;

    SECTION("default construction and basic types") {
        Json j; // null
        REQUIRE(j.is_null());

        Json n = 42;
        REQUIRE(n.is_number_integer());
        REQUIRE(n.get<int>() == 42);

        Json b = true;
        REQUIRE(b.is_boolean());
        REQUIRE(b.get<bool>() == true);

        Json s = "hello";
        REQUIRE(s.is_string());
        REQUIRE(s.get<std::string>() == "hello");
    }

    SECTION("object and array composition") {
        Json obj = Json::object();
        obj["answer"] = 42;
        obj["pi"] = 3.14;
        obj["ok"] = true;
        obj["name"] = "aetherion";

        REQUIRE(obj.is_object());
        REQUIRE(obj.at("answer").get<int>() == 42);
        REQUIRE(obj.at("ok").get<bool>() == true);
        REQUIRE(obj.at("name").get<std::string>() == "aetherion");

        Json arr = Json::array();
        arr.push_back(1);
        arr.push_back("two");
        arr.push_back(false);

        REQUIRE(arr.is_array());
        REQUIRE(arr.size() == 3);
        REQUIRE(arr.at(0).get<int>() == 1);
        REQUIRE(arr.at(1).get<std::string>() == "two");
        REQUIRE(arr.at(2).get<bool>() == false);
    }

    SECTION("parse + dump roundtrip") {
        const char* text = R"({
            "x": 1,
            "y": [2, 3],
            "name": "melina",
            "ok": true
        })";

        Json j = Json::parse(text);
        REQUIRE(j.is_object());
        REQUIRE(j.at("x").get<int>() == 1);
        REQUIRE(j.at("y").is_array());
        REQUIRE(j.at("y").size() == 2);
        REQUIRE(j.at("name").get<std::string>() == "melina");
        REQUIRE(j.at("ok").get<bool>() == true);

        const std::string dumped = j.dump();
        REQUIRE_FALSE(dumped.empty());

        Json k = Json::parse(dumped);
        REQUIRE(k == j);
    }
}
