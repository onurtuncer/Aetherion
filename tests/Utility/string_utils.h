// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------

#pragma once

#include <algorithm>
#include <cctype>
#include <string>
#include <string_view>
#include <vector>

namespace NasaCsv {

    inline std::string trim(std::string s) {
        auto is_ws = [](unsigned char c) { return std::isspace(c) != 0; };
        s.erase(s.begin(),
            std::find_if(s.begin(), s.end(), [&](char c) { return !is_ws((unsigned char)c); }));
        s.erase(
            std::find_if(s.rbegin(), s.rend(), [&](char c) { return !is_ws((unsigned char)c); }).base(),
            s.end());
        return s;
    }

    inline bool starts_with(std::string_view s, std::string_view prefix) {
        return s.size() >= prefix.size() && s.substr(0, prefix.size()) == prefix;
    }

    /// Normalizes a column name for case- and decoration-insensitive lookup.
    /// Steps applied in order:
    ///   1. Trim surrounding whitespace.
    ///   2. Strip bracketed unit suffixes: "phi (deg)" → "phi", "v [m/s]" → "v".
    ///   3. Lowercase.
    ///   4. Replace spaces, '/', '\', '-' with '_'.
    ///   5. Collapse consecutive underscores; strip leading/trailing underscores.
    inline std::string normalize_name(std::string s) {
        s = trim(std::move(s));

        auto strip_bracketed_tail = [&](char open, char close) {
            auto pos = s.find(open);
            if (pos == std::string::npos) return;
            auto end = s.find(close, pos + 1);
            if (end == std::string::npos) return;
            if (end + 1 == s.size() || trim(s.substr(end + 1)).empty())
                s = trim(s.substr(0, pos));
        };
        strip_bracketed_tail('(', ')');
        strip_bracketed_tail('[', ']');

        std::transform(s.begin(), s.end(), s.begin(),
            [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

        for (char& c : s)
            if (c == ' ' || c == '/' || c == '\\' || c == '-') c = '_';

        std::string out;
        out.reserve(s.size());
        bool prev_us = false;
        for (char c : s) {
            if (c == '_') { if (!prev_us) out.push_back(c); prev_us = true; }
            else          { out.push_back(c);               prev_us = false; }
        }
        while (!out.empty() && out.front() == '_') out.erase(out.begin());
        while (!out.empty() && out.back()  == '_') out.pop_back();
        return out;
    }

    /// Simple comma splitter — correct for purely numeric CSVs (no quoted commas).
    inline std::vector<std::string> split_csv_line(std::string_view line) {
        std::vector<std::string> out;
        std::string cur;
        cur.reserve(line.size());
        for (char ch : line) {
            if (ch == ',') {
                out.push_back(trim(cur));
                cur.clear();
            } else {
                cur.push_back(ch);
            }
        }
        out.push_back(trim(cur));
        return out;
    }

} // namespace NasaCsv
