// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------

#pragma once 

#include "nasa_csv.h"

#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>
#include <algorithm>
#include <cctype>

namespace NasaCsv {

    inline std::string normalize_name(std::string s) {
        // Lowercase, remove surrounding whitespace, remove common unit decorations.
        // Also replace spaces and slashes with underscores.
        s = trim(std::move(s));

        // Remove anything in (...) or [...] at the end: "phi (deg)" -> "phi"
        auto strip_bracketed_tail = [&](char open, char close) {
            auto pos = s.find(open);
            if (pos != std::string::npos) {
                auto end = s.find(close, pos + 1);
                if (end != std::string::npos) {
                    // if bracket part is near end, drop it
                    if (end + 1 == s.size() || trim(s.substr(end + 1)).empty()) {
                        s = trim(s.substr(0, pos));
                    }
                }
            }
            };
        strip_bracketed_tail('(', ')');
        strip_bracketed_tail('[', ']');

        // Lowercase
        std::transform(s.begin(), s.end(), s.begin(),
            [](unsigned char c) { return (char)std::tolower(c); });

        // Replace separators with '_'
        for (char& c : s) {
            if (c == ' ' || c == '/' || c == '\\' || c == '-') c = '_';
        }

        // Remove duplicate underscores
        std::string out;
        out.reserve(s.size());
        bool prev_us = false;
        for (char c : s) {
            if (c == '_') {
                if (!prev_us) out.push_back(c);
                prev_us = true;
            }
            else {
                out.push_back(c);
                prev_us = false;
            }
        }

        // Trim underscores
        while (!out.empty() && out.front() == '_') out.erase(out.begin());
        while (!out.empty() && out.back() == '_') out.pop_back();
        return out;
    }

    struct TableNorm {
        Table raw;
        std::vector<std::string> header_norm;
        std::unordered_map<std::string, std::size_t> col_index_norm;

        const std::vector<double>& col_norm(std::string_view normalized_name) const {
            auto it = col_index_norm.find(std::string{ normalized_name });
            if (it == col_index_norm.end())
                throw std::runtime_error("Missing normalized column: " + std::string{ normalized_name });
            return raw.cols[it->second];
        }
    };

    // Read + build normalized header mapping
    inline TableNorm read_numeric_csv_normalized(
        const std::string& path,
        bool has_header = true,
        char comment_char = '#'
    ) {
        TableNorm tn;
        tn.raw = read_numeric_csv(path, has_header, comment_char);

        tn.header_norm.resize(tn.raw.header.size());
        for (std::size_t i = 0; i < tn.raw.header.size(); ++i) {
            tn.header_norm[i] = normalize_name(tn.raw.header[i]);
            // If collisions happen (rare), later one overwrites; you can detect if you prefer.
            tn.col_index_norm[tn.header_norm[i]] = i;
        }
        return tn;
    }

    // Alias lookup: try multiple normalized names
    inline const std::vector<double>& col_any(
        const TableNorm& tn,
        const std::vector<std::string_view>& aliases_norm
    ) {
        for (auto a : aliases_norm) {
            auto it = tn.col_index_norm.find(std::string{ a });
            if (it != tn.col_index_norm.end())
                return tn.raw.cols[it->second];
        }

        // Helpful error message listing available columns
        std::string msg = "None of the aliases found. Tried: ";
        for (auto a : aliases_norm) { msg += std::string(a) + " "; }
        msg += "\nAvailable normalized columns:\n";
        for (auto& h : tn.header_norm) msg += "  - " + h + "\n";
        throw std::runtime_error(msg);
    }

} // namespace Aetherion::NasaCsv