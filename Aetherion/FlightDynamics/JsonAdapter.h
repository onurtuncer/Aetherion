// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX - License - Identifier: MIT
// License - Filename: LICENSE
// ------------------------------------------------------------------------------

#pragma once

#include <filesystem>
#include <string>
#include <string_view>
#include <vector>

namespace Aetherion::FlightDynamics {

	struct Json; 

	Json parse_json_file(const std::filesystem::path& path);

	bool        json_has(const Json& j, std::string_view key);
	Json        json_at(const Json& j, std::string_view key);
	double      json_get_number(const Json& j);
	bool        json_get_bool(const Json& j);
	std::string json_get_string(const Json& j);
	std::vector<Json> json_get_array(const Json& j);

} // namespace Aetherion::FlightDynamics
