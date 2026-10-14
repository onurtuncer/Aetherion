// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// Csv.h
//
// The CSV reader of the replay harness (sensor fusion step 6). Deliberately
// small: Hemerion's logs are plain comma-separated numbers with one header
// line, and the truth CSV adds a space after each comma and `[TYPE]`
// suffixes that LogContract::normalise() strips. The header is bound by
// name through LogContract::bindColumns(), so a renamed column fails loudly
// before any row is read.
//
// Cells: a number (strtod), `true`/`false` (1/0, the BOOL columns of the
// truth log), or empty/unparseable (NaN). Blank lines are skipped. A row
// with fewer cells than the header is padded with NaN; one with more is an
// error, because that is what a stray separator looks like.
// ------------------------------------------------------------------------------

#pragma once

#include <cstddef>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <istream>
#include <limits>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include <Aetherion/Estimation/Configuration.h>
#include <Aetherion/Estimation/LogContract.h>

namespace Aetherion::Estimation::Replay
{

struct CsvTable
{
  LogContract::ColumnMap columns;  ///< normalised name -> index
  std::vector<std::vector<double>> rows;

  [[nodiscard]] std::size_t column(std::string_view name) const
  {
    const auto it = columns.find(name);
    if (it == columns.end())
      throw std::out_of_range("CSV column not present: " + std::string(name));
    return it->second;
  }
  [[nodiscard]] bool has(std::string_view name) const { return columns.contains(name); }
  [[nodiscard]] double at(std::size_t row, std::string_view name) const { return rows[row][column(name)]; }
};

namespace detail
{
[[nodiscard]] inline double parseCell(std::string_view cell)
{
  cell = LogContract::detail::trim(cell);
  if (cell.empty())
    return std::numeric_limits<double>::quiet_NaN();
  if (cell == "true" || cell == "True" || cell == "TRUE")
    return 1.0;
  if (cell == "false" || cell == "False" || cell == "FALSE")
    return 0.0;
  const std::string s(cell);
  char* end = nullptr;
  const double v = std::strtod(s.c_str(), &end);
  if (end == s.c_str())
    return std::numeric_limits<double>::quiet_NaN();
  return v;
}
}  // namespace detail

/// Reads a whole CSV from `in`: header bound for (log, cfg), then rows.
[[nodiscard]] inline CsvTable
readCsv(std::istream& in, LogContract::Log log, const Configuration& cfg, std::string_view what = "csv")
{
  std::string line;
  if (!std::getline(in, line))
    throw std::runtime_error(std::string(what) + ": empty file");
  if (!line.empty() && static_cast<unsigned char>(line[0]) == 0xEF && line.size() >= 3)
    line.erase(0, 3);  // UTF-8 BOM

  CsvTable t;
  t.columns = LogContract::bindColumns(log, line, cfg);
  const std::size_t width = t.columns.size();

  std::size_t lineNo = 1;
  while (std::getline(in, line))
  {
    ++lineNo;
    if (LogContract::detail::trim(line).empty())
      continue;
    std::vector<double> row;
    row.reserve(width);
    std::string_view rest = line;
    for (;;)
    {
      const auto comma = rest.find(',');
      row.push_back(detail::parseCell(rest.substr(0, comma)));
      if (comma == std::string_view::npos)
        break;
      rest.remove_prefix(comma + 1);
    }
    if (row.size() > width)
      throw std::runtime_error(std::string(what) + ": line " + std::to_string(lineNo) + " has " +
                               std::to_string(row.size()) + " cells, header has " + std::to_string(width));
    row.resize(width, std::numeric_limits<double>::quiet_NaN());
    t.rows.push_back(std::move(row));
  }
  return t;
}

[[nodiscard]] inline CsvTable readCsvFile(const std::filesystem::path& path,
                                          LogContract::Log log,
                                          const Configuration& cfg)
{
  std::ifstream in(path);
  if (!in)
    throw std::runtime_error("cannot open " + path.string());
  return readCsv(in, log, cfg, path.filename().string());
}

}  // namespace Aetherion::Estimation::Replay
