// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------

#pragma once

#include "string_utils.h"  // normalize_name

#include <cstddef>
#include <stdexcept>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

namespace NasaCsv {

    /// Column-major in-memory table of doubles.
    /// Columns are addressed by name; rows are contiguous per column.
    struct Table {
        std::vector<std::string>                    header;
        std::unordered_map<std::string, std::size_t> col_index;
        std::vector<std::vector<double>>             cols; ///< cols[col][row]

        std::size_t rows()       const { return cols.empty() ? 0 : cols[0].size(); }
        std::size_t cols_count() const { return cols.size(); }

        const std::vector<double>& col(std::string_view name) const {
            auto it = col_index.find(std::string{name});
            if (it == col_index.end())
                throw std::runtime_error("Missing column: " + std::string{name});
            return cols[it->second];
        }
    };

    /// Table augmented with a normalized-name index for case- and unit-insensitive column lookup.
    struct TableNorm {
        Table                                        raw;
        std::vector<std::string>                     header_norm;
        std::unordered_map<std::string, std::size_t> col_index_norm;

        /// Look up a column by its pre-normalized name.
        const std::vector<double>& col_norm(std::string_view normalized_name) const {
            auto it = col_index_norm.find(std::string{normalized_name});
            if (it == col_index_norm.end())
                throw std::runtime_error(
                    "Missing normalized column: " + std::string{normalized_name});
            return raw.cols[it->second];
        }

        /// Look up a column by trying each alias in order (all pre-normalized).
        /// Throws a descriptive error listing all available columns if none match.
        const std::vector<double>& col_any(
            const std::vector<std::string_view>& aliases_norm
        ) const {
            for (auto a : aliases_norm) {
                auto it = col_index_norm.find(std::string{a});
                if (it != col_index_norm.end())
                    return raw.cols[it->second];
            }
            std::string msg = "None of the aliases found. Tried:";
            for (auto a : aliases_norm) { msg += ' '; msg += a; }
            msg += "\nAvailable normalized columns:\n";
            for (auto& h : header_norm) msg += "  - " + h + '\n';
            throw std::runtime_error(msg);
        }
    };

} // namespace NasaCsv
