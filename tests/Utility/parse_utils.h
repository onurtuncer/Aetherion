// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------

#pragma once

#include "string_utils.h"

#include <algorithm>
#include <charconv>
#include <optional>
#include <string>
#include <string_view>

namespace NasaCsv {

    /// Parses a trimmed string_view as a double.
    /// Returns std::nullopt for empty strings, "nan", and "null" tokens.
    /// Prefers std::from_chars (no locale, no allocation) when available;
    /// falls back to std::stod on older standard libraries.
    inline std::optional<double> parse_double(std::string_view s) {
        std::string str{ trim(std::string{s}) };
        if (str.empty()) return std::nullopt;

        auto lower = str;
        std::transform(lower.begin(), lower.end(), lower.begin(),
            [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
        if (lower == "nan" || lower == "null") return std::nullopt;

#if defined(__cpp_lib_to_chars) && __cpp_lib_to_chars >= 201611L
        double value{};
        const char* first = str.data();
        const char* last  = str.data() + str.size();
        auto res = std::from_chars(first, last, value, std::chars_format::general);
        if (res.ec == std::errc{} && res.ptr == last) return value;
        // fall through to stod on partial parse
#endif
        try {
            std::size_t idx = 0;
            double v = std::stod(str, &idx);
            if (idx != str.size()) return std::nullopt;
            return v;
        } catch (...) {
            return std::nullopt;
        }
    }

} // namespace NasaCsv
