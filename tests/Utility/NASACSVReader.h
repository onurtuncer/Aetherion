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
#include <charconv>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>
#include <vector>

namespace NasaCsv {

    // TODO [Onur] these first 3 functions to a seperate header

    inline std::string trim(std::string s) {
        auto is_ws = [](unsigned char c) { return std::isspace(c) != 0; };
        s.erase(s.begin(), std::find_if(s.begin(), s.end(), [&](char c) { return !is_ws((unsigned char)c); }));
        s.erase(std::find_if(s.rbegin(), s.rend(), [&](char c) { return !is_ws((unsigned char)c); }).base(), s.end());
        return s;
    }

    inline bool starts_with(std::string_view s, std::string_view p) {
        return s.size() >= p.size() && s.substr(0, p.size()) == p;
    }

    inline std::vector<std::string> split_csv_line(std::string_view line) {
        // Simple splitter: good for numeric CSV (no quoted commas).
        std::vector<std::string> out;
        std::string cur;
        cur.reserve(line.size());
        for (char ch : line) {
            if (ch == ',') {
                out.push_back(trim(cur));
                cur.clear();
            }
            else {
                cur.push_back(ch);
            }
        }
        out.push_back(trim(cur));
        return out;
    }

    // [Onur] This function in a seperate header

    inline std::optional<double> parse_double(std::string_view s) {
        // Accept "1.23", "-1e-3", etc. Reject empty/NaN tokens.
        // Use from_chars if possible; fallback to stod if needed.
        std::string str(trim(std::string{ s }));
        if (str.empty()) return std::nullopt;

        // Some CSVs use "NaN" or blanks; treat as missing.
        auto lower = str;
        std::transform(lower.begin(), lower.end(), lower.begin(), [](unsigned char c) { return (char)std::tolower(c); });
        if (lower == "nan" || lower == "null") return std::nullopt;

#if defined(__cpp_lib_to_chars) && __cpp_lib_to_chars >= 201611L
        double value{};
        auto first = str.data();
        auto last = str.data() + str.size();
        auto res = std::from_chars(first, last, value, std::chars_format::general);
        if (res.ec == std::errc{} && res.ptr == last) return value;
        // fall through
#endif
        try {
            size_t idx = 0;
            double v = std::stod(str, &idx);
            if (idx != str.size()) return std::nullopt;
            return v;
        }
        catch (...) {
            return std::nullopt;
        }
    }

    // TODO [Onur] Refactor this into a seperate header!
    struct Table {
        // Column-major storage: columns[name][row]
        std::vector<std::string> header;
        std::unordered_map<std::string, std::size_t> col_index;
        std::vector<std::vector<double>> cols; // size = ncol, each is nrow

        std::size_t rows() const { return cols.empty() ? 0 : cols[0].size(); }
        std::size_t cols_count() const { return cols.size(); }

        const std::vector<double>& col(std::string_view name) const {
            auto it = col_index.find(std::string{ name });
            if (it == col_index.end()) throw std::runtime_error("Missing column: " + std::string{ name });
            return cols[it->second];
        }
    };

    inline Table read_numeric_csv(
        const std::string& path,
        bool has_header = true,
        char comment_char = '#'
    ) {
        std::ifstream f(path);
        if (!f) throw std::runtime_error("Cannot open CSV: " + path);

        std::string line;
        std::vector<std::string> header;
        std::vector<std::vector<double>> cols;

        auto init_cols = [&](std::size_t n) {
            cols.assign(n, {});
            cols.shrink_to_fit();
            for (auto& c : cols) c.reserve(4096);
            };

        bool header_done = false;

        while (std::getline(f, line)) {
            auto t = trim(line);
            if (t.empty()) continue;
            if (!t.empty() && t[0] == comment_char) continue;

            auto fields = split_csv_line(t);
            if (!header_done && has_header) {
                header = fields;
                init_cols(header.size());
                header_done = true;
                continue;
            }

            if (!header_done) {
                // No header: create generic names c0,c1...
                header.resize(fields.size());
                for (std::size_t i = 0; i < fields.size(); ++i) header[i] = "c" + std::to_string(i);
                init_cols(fields.size());
                header_done = true;
            }

            if (fields.size() != header.size())
                throw std::runtime_error("CSV row has different column count in: " + path);

            for (std::size_t i = 0; i < fields.size(); ++i) {
                auto v = parse_double(fields[i]);
                if (!v) {
                    throw std::runtime_error("Non-numeric/missing value at col " + std::to_string(i) + " in: " + path);
                }
                cols[i].push_back(*v);
            }
        }

        Table tbl;
        tbl.header = header;
        tbl.cols = std::move(cols);
        for (std::size_t i = 0; i < tbl.header.size(); ++i) {
            tbl.col_index[tbl.header[i]] = i;
        }
        return tbl;
    }

} // NasaCsv