// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// GoldenVectors.h
//
// Golden vectors of the reference filter (sensor fusion step 8;
// doc/codegen.rst): a recorded sequence of InvariantEKF calls -- the inputs
// of every predict and update, and the state and covariance the reference
// filter holds after each -- with the tolerances an implementation is held
// to and the configuration hash. The files live in
// data/Estimation/GoldenVectors/ and are data: Hemerion replays them against
// its embedded filter (its step 9) and refuses a file whose hash differs
// from the one compiled into its generated code.
//
// This header is the file format (read, write), the reference run that
// produces the outputs from the inputs (runReference) and the comparison
// under the file's tolerances (compare).
//
// File format: text, one item per line, comma-separated, '#' starts a comment
// line. Numbers are printed with 17 significant digits, which round-trips a
// double exactly. Header items, each once, in this order:
//
//   format,aetherion.estimation.golden/1
//   configuration,<name>
//   convention,<left-invariant|right-invariant>
//   hash,0x<16 hex digits>              FNV-1a 64 of the configuration descriptor
//   error_dim,<n>
//   noise_dim,<m>
//   state_dim,22
//   tolerance,<set>,<quantity>,<abs>,<rel>   one line per quantity (see compare)
//   frame,<12>                          r_W0 (3), R_WL (9, row-major)
//   qc_diag,<m>                         diagonal of the continuous noise PSD
//   initial,<state 22>,<P n(n+1)/2>
//
// then the records, in the order the filter is called. Each is an input line,
//
//   predict,<gyro 3>,<accel 3>,<dt>
//   gnss_position,<y 3, LCI>,<Sigma 9, LCI, row-major>
//   gnss_velocity,<y 3, LCI>,<Sigma 9, LCI, row-major>
//   barometer,<y, pressure altitude m>,<sigma m>
//
// followed, for an update, by what the reference computed from it,
//
//   innovation,<nis>,<z (measurement dimension)>
//
// and always by the filter's state after the call,
//
//   state,<state 22>,<P n(n+1)/2>
//
// The state is packed as in the generated code (codegen/EkfMaps.h,
// StateLayout): R row-major (9), v, p, b_g, b_a, b_p. P is the upper triangle,
// row by row (P is symmetric: the reference symmetrises it after every call),
// in the convention's error coordinates and the configuration's block order.
// ------------------------------------------------------------------------------

#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <istream>
#include <map>
#include <ostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include <Eigen/Dense>

#include <Aetherion/Estimation/EKF.h>

#include "../codegen/EkfMaps.h"

namespace Aetherion::Estimation::Golden {

    inline constexpr std::string_view kFormat = "aetherion.estimation.golden/1";

    /// A deviation d from a reference of magnitude r passes if d <= abs + rel * r.
    struct Tolerance
    {
        double abs = 0.0;
        double rel = 0.0;
    };

    /// The quantities compare() checks, each under the tolerance of the same name.
    inline constexpr std::array<std::string_view, 9> kQuantities{ "attitude", "velocity", "position", "gyro_bias",
        "accel_bias", "baro_bias", "covariance", "innovation", "nis" };

    struct Record
    {
        std::string kind;               ///< predict, gnss_position, gnss_velocity or barometer
        std::vector<double> input;      ///< the input line's numbers
        double nis = 0.0;               ///< updates only
        std::vector<double> z;          ///< updates only: the innovation
        std::vector<double> state;      ///< packed state after the call
        std::vector<double> covariance; ///< upper triangle of P after the call
    };

    struct File
    {
        std::string configuration;
        std::string convention;
        std::uint64_t hash = 0;
        int errorDim = 0;
        int noiseDim = 0;
        int stateDim = 0;
        std::map<std::string, std::map<std::string, Tolerance>> tolerances; ///< set -> quantity -> tolerance
        std::vector<double> frame;
        std::vector<double> qcDiag;
        std::vector<double> state0;
        std::vector<double> covariance0;
        std::vector<Record> records;
    };

    [[nodiscard]] constexpr bool isUpdate(std::string_view kind) { return kind != "predict"; }

    /// Number of input values of a record kind, or -1 if the kind is unknown.
    [[nodiscard]] constexpr int inputSize(std::string_view kind)
    {
        if (kind == "predict") return 7;
        if (kind == "gnss_position" || kind == "gnss_velocity") return 12;
        if (kind == "barometer") return 2;
        return -1;
    }

    [[nodiscard]] constexpr int measurementDim(std::string_view kind) { return kind == "barometer" ? 1 : 3; }

    // ---- packing of P ------------------------------------------------------------

    [[nodiscard]] inline std::vector<double> packUpper(const Eigen::MatrixXd& P)
    {
        std::vector<double> u;
        for (Eigen::Index i = 0; i < P.rows(); ++i)
            for (Eigen::Index j = i; j < P.cols(); ++j) u.push_back(P(i, j));
        return u;
    }

    [[nodiscard]] inline Eigen::MatrixXd unpackUpper(const std::vector<double>& u, int n)
    {
        if (static_cast<int>(u.size()) != n * (n + 1) / 2)
            throw std::runtime_error("golden vectors: covariance size mismatch");
        Eigen::MatrixXd P(n, n);
        std::size_t k = 0;
        for (int i = 0; i < n; ++i)
            for (int j = i; j < n; ++j) {
                P(i, j) = u[k];
                P(j, i) = u[k++];
            }
        return P;
    }

    // ---- write -------------------------------------------------------------------

    namespace detail {
        inline void put(std::ostream& os, const std::vector<double>& v)
        {
            char buffer[40];
            for (const double d : v) {
                std::snprintf(buffer, sizeof buffer, ",%.17g", d);
                os << buffer;
            }
        }

        inline std::string hex(std::uint64_t h)
        {
            char buffer[24];
            std::snprintf(buffer, sizeof buffer, "0x%016llx", static_cast<unsigned long long>(h));
            return buffer;
        }

        /// The fields of a line; the first is the tag.
        inline std::vector<std::string> split(const std::string& line)
        {
            std::vector<std::string> fields;
            std::string field;
            std::istringstream s(line);
            while (std::getline(s, field, ',')) fields.push_back(field);
            return fields;
        }

        inline double number(const std::string& s)
        {
            char* end = nullptr;
            const double d = std::strtod(s.c_str(), &end);
            if (end == s.c_str() || *end != '\0') throw std::runtime_error("golden vectors: not a number: '" + s + "'");
            return d;
        }

        inline std::vector<double> numbers(const std::vector<std::string>& fields, std::size_t first, std::size_t count)
        {
            if (fields.size() < first + count)
                throw std::runtime_error("golden vectors: line '" + fields.front() + "' is too short");
            std::vector<double> v;
            for (std::size_t i = first; i < first + count; ++i) v.push_back(number(fields[i]));
            return v;
        }
    } // namespace detail

    inline void write(std::ostream& os, const File& f)
    {
        os << "# Aetherion golden vectors of the reference invariant EKF (TODO-sensor-fusion.md, step 8).\n"
           << "# Recorded inputs and outputs of InvariantEKF<double> for one configuration and error\n"
           << "# convention. Format and comparison rule: tests/Estimation/golden/GoldenVectors.h and\n"
           << "# doc/codegen.rst. Data; written by generate_golden_vectors, never edited by hand.\n"
           << "#\n"
           << "# state: R row-major (9), v (3), p (3), gyro bias (3), accel bias (3), baro bias (1).\n"
           << "# P: upper triangle, row by row, in the convention's error coordinates.\n"
           << "# tolerance,<set>,<quantity>,<abs>,<rel>: a deviation d passes if d <= abs + rel * r, with\n"
           << "#   d, r the largest absolute deviation and reference entry of a state block, of z or of nis;\n"
           << "#   for the covariance, entry by entry, r = sqrt(P_ii P_jj) of the reference.\n"
           << "format," << kFormat << "\n"
           << "configuration," << f.configuration << "\n"
           << "convention," << f.convention << "\n"
           << "hash," << detail::hex(f.hash) << "\n"
           << "error_dim," << f.errorDim << "\n"
           << "noise_dim," << f.noiseDim << "\n"
           << "state_dim," << f.stateDim << "\n";
        for (const auto& [set, quantities] : f.tolerances)
            for (const std::string_view q : kQuantities) {
                os << "tolerance," << set << "," << q;
                detail::put(os, { quantities.at(std::string(q)).abs, quantities.at(std::string(q)).rel });
                os << "\n";
            }
        os << "frame";
        detail::put(os, f.frame);
        os << "\nqc_diag";
        detail::put(os, f.qcDiag);
        os << "\ninitial";
        detail::put(os, f.state0);
        detail::put(os, f.covariance0);
        os << "\n";
        for (const Record& r : f.records) {
            os << r.kind;
            detail::put(os, r.input);
            os << "\n";
            if (isUpdate(r.kind)) {
                os << "innovation";
                detail::put(os, { r.nis });
                detail::put(os, r.z);
                os << "\n";
            }
            os << "state";
            detail::put(os, r.state);
            detail::put(os, r.covariance);
            os << "\n";
        }
    }

    // ---- read --------------------------------------------------------------------

    namespace detail {
        inline void readHeaderLine(File& f, const std::vector<std::string>& fields)
        {
            const std::string& tag = fields.front();
            const auto text = [&fields]() {
                if (fields.size() != 2)
                    throw std::runtime_error("golden vectors: malformed line '" + fields.front() + "'");
                return fields[1];
            };
            const auto triangle = static_cast<std::size_t>(f.errorDim * (f.errorDim + 1) / 2);
            if (tag == "format") {
                if (text() != kFormat) throw std::runtime_error("golden vectors: unknown format " + text());
            } else if (tag == "configuration")
                f.configuration = text();
            else if (tag == "convention")
                f.convention = text();
            else if (tag == "hash")
                f.hash = std::stoull(text(), nullptr, 16);
            else if (tag == "error_dim")
                f.errorDim = std::stoi(text());
            else if (tag == "noise_dim")
                f.noiseDim = std::stoi(text());
            else if (tag == "state_dim")
                f.stateDim = std::stoi(text());
            else if (tag == "tolerance") {
                const std::vector<double> t = numbers(fields, 3, 2);
                f.tolerances[fields[1]][fields[2]] = { t[0], t[1] };
            } else if (tag == "frame")
                f.frame = numbers(fields, 1, fields.size() - 1);
            else if (tag == "qc_diag")
                f.qcDiag = numbers(fields, 1, fields.size() - 1);
            else if (tag == "initial") {
                f.state0 = numbers(fields, 1, static_cast<std::size_t>(f.stateDim));
                f.covariance0 = numbers(fields, 1 + static_cast<std::size_t>(f.stateDim), triangle);
            } else
                throw std::runtime_error("golden vectors: unknown line '" + tag + "'");
        }

        inline void checkHeader(const File& f)
        {
            if (f.stateDim != CodeGen::StateLayout::kDim || f.errorDim <= 0 || f.hash == 0 ||
                static_cast<int>(f.frame.size()) != CodeGen::FrameLayout::kDim ||
                static_cast<int>(f.qcDiag.size()) != f.noiseDim || f.state0.empty())
                throw std::runtime_error("golden vectors: incomplete or inconsistent header");
        }
    } // namespace detail

    [[nodiscard]] inline File read(std::istream& is)
    {
        File f;
        Record* open = nullptr; // the record whose output lines are still to come
        std::string line;
        while (std::getline(is, line)) {
            if (!line.empty() && line.back() == '\r') line.pop_back();
            if (line.empty() || line.front() == '#') continue;
            const std::vector<std::string> fields = detail::split(line);
            const std::string& tag = fields.front();
            const auto n = static_cast<std::size_t>(f.errorDim);
            if (inputSize(tag) >= 0) {
                if (open != nullptr)
                    throw std::runtime_error("golden vectors: record without a state line before '" + tag + "'");
                detail::checkHeader(f);
                f.records.push_back({ tag, detail::numbers(fields, 1, static_cast<std::size_t>(inputSize(tag))) });
                open = &f.records.back();
            } else if (tag == "innovation" && open != nullptr && isUpdate(open->kind)) {
                open->nis = detail::number(fields.at(1));
                open->z = detail::numbers(fields, 2, static_cast<std::size_t>(measurementDim(open->kind)));
            } else if (tag == "state" && open != nullptr) {
                open->state = detail::numbers(fields, 1, static_cast<std::size_t>(f.stateDim));
                open->covariance = detail::numbers(fields, 1 + static_cast<std::size_t>(f.stateDim), n * (n + 1) / 2);
                open = nullptr;
            } else if (open == nullptr && f.records.empty())
                detail::readHeaderLine(f, fields);
            else
                throw std::runtime_error("golden vectors: unexpected line '" + tag + "'");
        }
        if (open != nullptr) throw std::runtime_error("golden vectors: the last record has no state line");
        detail::checkHeader(f);
        return f;
    }

    // ---- the reference run -------------------------------------------------------

    /// The process-noise densities whose PSD diagonal is qcDiag. Throws if qcDiag
    /// is not one the reference filter can be given (the entries of a block
    /// differ, or the square root does not square back exactly).
    [[nodiscard]] inline ProcessNoise<double> processNoise(const Configuration& cfg, const std::vector<double>& qcDiag)
    {
        const auto density = [&](int offset) {
            return offset < 0 ? 0.0 : std::sqrt(qcDiag.at(static_cast<std::size_t>(offset)));
        };
        ProcessNoise<double> q;
        q.gyroNoise = density(0);
        q.accelNoise = density(3);
        q.gyroBiasWalk = density(noiseOffset(cfg, BlockId::GyroBias));
        q.accelBiasWalk = density(noiseOffset(cfg, BlockId::AccelBias));
        q.baroBiasWalk = density(noiseOffset(cfg, BlockId::BaroBias));
        const Eigen::MatrixXd Qc = StrapdownProcessModel<double>(LciFrame<double>{}).continuousNoiseCovariance(cfg, q);
        for (int i = 0; i < Qc.rows(); ++i)
            if (Qc(i, i) != qcDiag.at(static_cast<std::size_t>(i)))
                throw std::runtime_error("golden vectors: qc_diag is not a process-noise PSD of the reference filter");
        return q;
    }

    /// Run the reference filter InvariantEKF<double, C> over the inputs of
    /// `inputs` (header, initial state and every record's input line) and return
    /// the file with every output filled in from that run.
    template <ErrorConvention C>
    [[nodiscard]] File runReference(const Configuration& cfg, File inputs)
    {
        if (cfg.dimension() != inputs.errorDim || noiseDimension(cfg) != inputs.noiseDim)
            throw std::runtime_error("golden vectors: the file's dimensions are not the configuration's");
        const LciFrame<double> frame = CodeGen::readFrame(inputs.frame, 0);
        InvariantEKF<double, C> ekf(cfg, frame, processNoise(cfg, inputs.qcDiag));
        ekf.initialise(CodeGen::readState(inputs.state0, 0), unpackUpper(inputs.covariance0, inputs.errorDim));

        for (Record& r : inputs.records) {
            const std::vector<double>& in = r.input;
            UpdateResult<double> u;
            Eigen::VectorXd z;
            Eigen::MatrixXd H, R;
            const auto& models = ekf.measurementModels();
            if (r.kind == "predict") {
                ekf.predict({ CodeGen::readVec3(in, 0), CodeGen::readVec3(in, 3) }, in[6]);
            } else if (r.kind == "gnss_position") {
                models.template gnssPosition<C>(
                    cfg, ekf.state(), CodeGen::readVec3(in, 0), CodeGen::readMat3(in, 3), z, H, R);
                u = ekf.updateGnssPosition(CodeGen::readVec3(in, 0), CodeGen::readMat3(in, 3));
            } else if (r.kind == "gnss_velocity") {
                models.template gnssVelocity<C>(
                    cfg, ekf.state(), CodeGen::readVec3(in, 0), CodeGen::readMat3(in, 3), z, H, R);
                u = ekf.updateGnssVelocity(CodeGen::readVec3(in, 0), CodeGen::readMat3(in, 3));
            } else {
                models.template barometer<C>(cfg, ekf.state(), in[0], in[1], z, H, R);
                u = ekf.updateBarometer(in[0], in[1]);
            }
            if (isUpdate(r.kind)) {
                if (!u.applied)
                    throw std::runtime_error(
                        "golden vectors: the reference filter did not apply a " + r.kind + " update");
                r.nis = u.nis;
                r.z.assign(z.data(), z.data() + z.size());
            }
            r.state = CodeGen::packState(ekf.state());
            r.covariance = packUpper(ekf.covariance());
        }
        return inputs;
    }

    // ---- comparison --------------------------------------------------------------

    /// The worst case of one quantity over a comparison.
    struct Worst
    {
        double ratio = 0.0;     ///< largest deviation / allowed deviation; passes if <= 1
        double deviation = 0.0; ///< the deviation at that ratio
        double reference = 0.0; ///< the reference magnitude it is relative to
        int record = -1;        ///< index of the record (-1: none compared)
    };

    struct Comparison
    {
        std::map<std::string, Worst> worst; ///< by quantity
        [[nodiscard]] bool passed() const
        {
            return std::all_of(worst.begin(), worst.end(), [](const auto& w) { return w.second.ratio <= 1.0; });
        }
        [[nodiscard]] std::string summary() const
        {
            std::ostringstream s;
            for (const auto& [q, w] : worst)
                s << q << ": ratio " << w.ratio << " (deviation " << w.deviation << ", reference " << w.reference
                  << ", record " << w.record << "); ";
            return s.str();
        }
    };

    namespace detail {
        inline void note(Comparison& c, std::string_view quantity, const Tolerance& t, double d, double r, int record)
        {
            // A deviation that is not a number must fail, not compare false.
            const double allowed = t.abs + t.rel * r;
            const double ratio =
                std::isfinite(d) ? (allowed > 0.0 ? d / allowed : (d > 0.0 ? HUGE_VAL : 0.0)) : HUGE_VAL;
            Worst& w = c.worst[std::string(quantity)];
            if (w.record < 0 || ratio > w.ratio) w = { ratio, d, r, record };
        }

        /// Largest absolute deviation and largest reference entry of a run of values.
        inline void block(Comparison& c, std::string_view quantity, const Tolerance& t, const double* test,
            const double* ref, int size, int record)
        {
            double d = 0.0, r = 0.0;
            for (int i = 0; i < size; ++i) {
                const double e = std::abs(test[i] - ref[i]);
                d = std::isfinite(e) ? std::max(d, e) : e;
                if (!std::isfinite(e)) break;
                r = std::max(r, std::abs(ref[i]));
            }
            note(c, quantity, t, d, r, record);
        }
    } // namespace detail

    /// Compare the outputs of `test` with those of `reference` under the
    /// reference file's tolerance set. The two must hold the same records.
    [[nodiscard]] inline Comparison compare(const File& reference, const File& test, const std::string& set = "float64")
    {
        namespace L = CodeGen::StateLayout;
        const std::map<std::string, Tolerance>& tol = reference.tolerances.at(set);
        if (reference.records.size() != test.records.size())
            throw std::runtime_error("golden vectors: record counts differ");
        const int n = reference.errorDim;
        Comparison c;
        for (std::size_t k = 0; k < reference.records.size(); ++k) {
            const Record& a = test.records[k];
            const Record& b = reference.records[k];
            const int rec = static_cast<int>(k);
            if (a.kind != b.kind || a.state.size() != b.state.size() || a.covariance.size() != b.covariance.size() ||
                a.z.size() != b.z.size())
                throw std::runtime_error("golden vectors: records differ in kind or size");
            const auto state = [&](std::string_view q, int offset, int size) {
                detail::block(
                    c, q, tol.at(std::string(q)), a.state.data() + offset, b.state.data() + offset, size, rec);
            };
            state("attitude", L::kAttitude, 9);
            state("velocity", L::kVelocity, 3);
            state("position", L::kPosition, 3);
            state("gyro_bias", L::kGyroBias, 3);
            state("accel_bias", L::kAccelBias, 3);
            state("baro_bias", L::kBaroBias, 1);

            const Eigen::MatrixXd Pa = unpackUpper(a.covariance, n);
            const Eigen::MatrixXd Pb = unpackUpper(b.covariance, n);
            for (int i = 0; i < n; ++i)
                for (int j = i; j < n; ++j)
                    detail::note(c, "covariance", tol.at("covariance"), std::abs(Pa(i, j) - Pb(i, j)),
                        std::sqrt(Pb(i, i) * Pb(j, j)), rec);

            if (isUpdate(b.kind)) {
                detail::block(
                    c, "innovation", tol.at("innovation"), a.z.data(), b.z.data(), static_cast<int>(b.z.size()), rec);
                detail::block(c, "nis", tol.at("nis"), &a.nis, &b.nis, 1, rec);
            }
        }
        return c;
    }

} // namespace Aetherion::Estimation::Golden
