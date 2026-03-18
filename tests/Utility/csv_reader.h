// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------

#pragma once

#include "parse_utils.h"
#include "string_utils.h"
#include "table.h"

#include <cstddef>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace NasaCsv {

    /// Reads a numeric CSV file into a Table.
    ///
    /// @param path         Path to the CSV file.
    /// @param has_header   If true, the first non-comment line is treated as a header row.
    ///                     If false, columns are named c0, c1, …
    /// @param comment_char Lines whose first non-whitespace character matches this are skipped.
    ///
    /// @throws std::runtime_error on I/O failure, column-count mismatch, or non-numeric values.
    inline Table read_numeric_csv(
        const std::string& path,
        bool  has_header   = true,
        char  comment_char = '#'
    ) {
        std::ifstream f(path);
        if (!f) throw std::runtime_error("Cannot open CSV: " + path);

        std::vector<std::string>         header;
        std::vector<std::vector<double>> cols;
        bool header_done = false;

        auto init_cols = [&](std::size_t n) {
            cols.assign(n, {});
            for (auto& c : cols) c.reserve(4096);
        };

        std::string line;
        while (std::getline(f, line)) {
            auto t = trim(line);
            if (t.empty() || t[0] == comment_char) continue;

            auto fields = split_csv_line(t);

            if (!header_done) {
                if (has_header) {
                    header = fields;
                } else {
                    header.resize(fields.size());
                    for (std::size_t i = 0; i < fields.size(); ++i)
                        header[i] = "c" + std::to_string(i);
                }
                init_cols(header.size());
                header_done = true;
                if (has_header) continue; // header row carries no data
            }

            if (fields.size() != header.size())
                throw std::runtime_error("CSV row has different column count in: " + path);

            for (std::size_t i = 0; i < fields.size(); ++i) {
                auto v = parse_double(fields[i]);
                if (!v)
                    throw std::runtime_error(
                        "Non-numeric/missing value at col " + std::to_string(i) + " in: " + path);
                cols[i].push_back(*v);
            }
        }

        Table tbl;
        tbl.header = std::move(header);
        tbl.cols   = std::move(cols);
        for (std::size_t i = 0; i < tbl.header.size(); ++i)
            tbl.col_index[tbl.header[i]] = i;
        return tbl;
    }

    /// Reads a numeric CSV and builds a normalized-name index for each column header.
    ///
    /// Normalization strips unit decorations ("phi (deg)" → "phi"), lowercases, and
    /// replaces separator characters with underscores. See normalize_name() for the
    /// full algorithm.
    ///
    /// @throws std::runtime_error if two distinct raw headers normalize to the same name,
    ///         which would otherwise cause a silent data-loss bug.
    inline TableNorm read_numeric_csv_normalized(
        const std::string& path,
        bool has_header   = true,
        char comment_char = '#'
    ) {
        TableNorm tn;
        tn.raw = read_numeric_csv(path, has_header, comment_char);

        const std::size_t n = tn.raw.header.size();
        tn.header_norm.resize(n);

        for (std::size_t i = 0; i < n; ++i) {
            tn.header_norm[i] = normalize_name(tn.raw.header[i]);

            auto [it, inserted] = tn.col_index_norm.emplace(tn.header_norm[i], i);
            if (!inserted)
                throw std::runtime_error(
                    "Normalized name collision: '" + tn.raw.header[it->second] +
                    "' and '" + tn.raw.header[i] +
                    "' both normalize to '" + tn.header_norm[i] + "' in: " + path);
        }
        return tn;
    }

} // namespace NasaCsv
