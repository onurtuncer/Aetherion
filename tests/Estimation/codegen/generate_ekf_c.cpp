// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// generate_ekf_c: the step-8 generator tool (TODO-sensor-fusion.md, step 8;
// doc/codegen.rst). For one configuration and one error convention it tapes
// the maps of EkfMaps.h with CppAD::AD<CppAD::cg::CG<double>> and writes
//
//   <out>/<stem>.c   plain C (C99, <math.h> only) for f (propagate), its
//                    Jacobian side (transition: Phi, Qd), h and its Jacobian
//                    for every measurement the configuration carries
//                    (z, H, R), and retract;
//   <out>/<stem>.h   prototypes, the error-state dimension, the block offsets,
//                    the input/output layout of every function and the
//                    configuration hash.
//
//   generate_ekf_c --out <dir> --banner <file> --generator-commit <sha>
//                  --cppadcg-version <version>
//                  [--config nav-baro-gnss16] [--convention ri|li]
//                  [--stem aeth_ekf]
//
// The banner (project, copyright, SPDX identifier) is an input, never
// hard-coded: the generated files are not this project's to license
// (TODO-sensor-fusion.md, "Decided (2026-11-13)"). They are never committed
// to, installed from or released with this repository; the recipient supplies
// its own banner. The tool adds the lines that name the generator, its
// commit, CppADCodeGen's version and the configuration hash, and a marker
// line that cmake/CheckGeneratedEstimatorCodeNotPublished.cmake searches for.
//
// The convention defaults to Configuration.h's kErrorConvention, the one the
// pinned hash names. The functions are double precision.
// ------------------------------------------------------------------------------

// CppADCodeGen must precede every CppAD include; see generate_estimation_cg.cpp
// and CppADCodeGenCore.h for the order and for why <cppad/cg.hpp> is not used.
#include "CppADCodeGenCore.h"
// clang-format off
#include <cppad/example/cppad_eigen.hpp>
#include <cppad/cg/support/cppadcg_eigen.hpp>
// clang-format on

#include <algorithm>
#include <cctype>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <numbers>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include "EkfMaps.h"

namespace {
    using CGD = CppAD::cg::CG<double>;
    using ADCG = CppAD::AD<CGD>;
    namespace CG = Aetherion::Estimation::CodeGen;
    namespace Est = Aetherion::Estimation;

    struct Options
    {
        std::filesystem::path outDir;
        std::filesystem::path bannerFile;
        std::string generatorCommit;
        std::string cppadcgVersion;
        std::string configName = "nav-baro-gnss16";
        std::string stem = "aeth_ekf";
        Est::ErrorConvention convention = Est::kErrorConvention;
    };

    constexpr std::string_view kUsage =
        "usage: generate_ekf_c --out <dir> --banner <file> --generator-commit <sha> --cppadcg-version <version>\n"
        "                      [--config nav-baro-gnss16] [--convention ri|li] [--stem aeth_ekf]\n";

    /// The configurations the tool knows by name.
    const Est::Configuration& configuration(const std::string& configName)
    {
        if (configName == "nav-baro-gnss16") return Est::kNavBaroGnss16;
        throw std::invalid_argument("unknown configuration '" + configName + "' (known: nav-baro-gnss16)");
    }

    bool isIdentifier(const std::string& s)
    {
        if (s.empty() || std::isdigit(static_cast<unsigned char>(s.front())) != 0) return false;
        return std::all_of(s.begin(), s.end(), [](unsigned char c) { return std::isalnum(c) != 0 || c == '_'; });
    }

    Options parse(int argc, char** argv)
    {
        std::map<std::string, std::string> args;
        for (int i = 1; i < argc; i += 2) {
            const std::string key = argv[i];
            if (key.rfind("--", 0) != 0 || i + 1 >= argc)
                throw std::invalid_argument("expected '--option value', got '" + key + "'");
            args[key.substr(2)] = argv[i + 1];
        }
        auto take = [&args](const std::string& key, bool required, std::string fallback = {}) {
            const auto it = args.find(key);
            if (it == args.end()) {
                if (required) throw std::invalid_argument("missing --" + key);
                return fallback;
            }
            std::string value = it->second;
            args.erase(it);
            return value;
        };

        Options o;
        o.outDir = take("out", true);
        o.bannerFile = take("banner", true);
        o.generatorCommit = take("generator-commit", true);
        o.cppadcgVersion = take("cppadcg-version", true);
        o.configName = take("config", false, o.configName);
        o.stem = take("stem", false, o.stem);
        const std::string convention = take("convention", false);
        if (convention == "ri")
            o.convention = Est::ErrorConvention::RightInvariant;
        else if (convention == "li")
            o.convention = Est::ErrorConvention::LeftInvariant;
        else if (!convention.empty())
            throw std::invalid_argument("--convention must be ri or li");
        if (!args.empty()) throw std::invalid_argument("unknown option --" + args.begin()->first);
        if (!isIdentifier(o.stem)) throw std::invalid_argument("--stem must be a C identifier");
        return o;
    }

    std::string upper(std::string_view s)
    {
        std::string u(s);
        std::transform(
            u.begin(), u.end(), u.begin(), [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
        return u;
    }

    std::string hashString(std::uint64_t hash)
    {
        std::ostringstream s;
        s << "0x" << std::hex << std::setw(16) << std::setfill('0') << hash;
        return s.str();
    }

    /// The marker every generated file carries, assembled so that this source
    /// file does not contain it: the not-published check greps tracked files.
    std::string marker() { return std::string("AETHERION-GENERATED-") + "ESTIMATOR-CODE"; }

    /// The caller's banner as a C comment, followed by the provenance lines.
    std::string fileComment(const Options& o, const Est::Configuration& cfg)
    {
        std::ifstream in(o.bannerFile);
        if (!in) throw std::runtime_error("cannot read banner file " + o.bannerFile.string());
        std::ostringstream c;
        c << "/*\n";
        std::string line;
        bool any = false;
        while (std::getline(in, line)) {
            if (!line.empty() && line.back() == '\r') line.pop_back();
            if (line.find("*/") != std::string::npos) throw std::runtime_error("the banner must not contain '*/'");
            c << (line.empty() ? " *" : " * ") << line << "\n";
            any = true;
        }
        if (!any) throw std::runtime_error("the banner file is empty");
        c << " *\n"
          << " * " << marker() << "\n"
          << " * Generated file. Do not edit; change the generator and regenerate.\n"
          << " * Generator: Aetherion generate_ekf_c, commit " << o.generatorCommit << "\n"
          << " * CppADCodeGen: " << o.cppadcgVersion << "\n"
          << " * Configuration: " << o.configName << ", " << Est::toString(o.convention) << ", hash "
          << hashString(cfg.hash(o.convention)) << "\n"
          << " * Not to be committed to, installed from or released with the Aetherion\n"
          << " * repository.\n"
          << " */\n";
        return c.str();
    }

    /// A point at which every map is taped. Its values are not frozen into the
    /// code: every operation on the path is recorded, and the branches are
    /// CondExp, which the generated C evaluates at run time.
    std::vector<double> tapingPoint(const Est::Configuration& cfg, CG::Function f)
    {
        constexpr double deg = std::numbers::pi / 180.0;
        Est::EstimationState<double> x;
        x.X.R = Aetherion::ODE::RKMK::Lie::SO3::Exp_R(Eigen::Vector3d(0.3, -0.2, 1.1));
        x.X.v = Eigen::Vector3d(120.0, 40.0, -5.0);
        x.X.p = Eigen::Vector3d(8000.0, -3000.0, -1500.0);
        x.gyroBias = Eigen::Vector3d(0.002, -0.001, 0.0015);
        x.accelBias = Eigen::Vector3d(0.05, -0.02, 0.03);
        x.baroBias = 12.0;
        const std::vector<double> frame =
            CG::packFrame(Est::LciFrame<double>::FromGeodetic(36.0192 * deg, -75.6744 * deg, 3.0, 0.3));

        std::vector<double> in;
        for (const CG::Segment& s : CG::inputSegments(cfg, f)) {
            if (s.name == "state")
                CG::appendState(x, in);
            else if (s.name == "frame")
                in.insert(in.end(), frame.begin(), frame.end());
            else if (s.name == "dt")
                in.push_back(0.01);
            else if (s.name == "meas_cov")
                in.insert(in.end(), { 2.25, 0.0, 0.0, 0.0, 2.25, 0.0, 0.0, 0.0, 9.0 });
            else
                for (int i = 0; i < s.size; ++i) in.push_back(0.1 + 0.01 * i);
        }
        return in;
    }

    struct Generated
    {
        std::string source;
        std::size_t temporaries = 0;
        std::size_t tapeVariables = 0;
    };

    /// Tape function f and write it as `void <symbol>(const double* x, double* y)`.
    Generated generate(const Options& o, const Est::Configuration& cfg, CG::Function f, const std::string& symbol)
    {
        const std::vector<double> x0 = tapingPoint(cfg, f);
        std::vector<ADCG> ax(x0.size());
        for (std::size_t i = 0; i < x0.size(); ++i) ax[i] = ADCG(CGD(x0[i]));
        CppAD::Independent(ax);
        const std::vector<ADCG> ay = CG::evaluate<ADCG>(cfg, o.convention, f, ax);
        CppAD::ADFun<CGD> fun(ax, ay);

        CppAD::cg::CodeHandler<double> handler;
        std::vector<CGD> x(x0.size());
        handler.makeVariables(x);
        std::vector<CGD> y = fun.Forward(0, x);

        CppAD::cg::LanguageC<double> lang("double");
        // 17 significant digits reproduce every double constant exactly; the
        // default, 15, does not (TODO-sensor-fusion.md, "Things to check").
        lang.setParameterPrecision(std::numeric_limits<double>::max_digits10);
        CppAD::cg::LangCDefaultVariableNameGenerator<double> nameGen("y", "x", "v");
        std::ostringstream body;
        handler.generateCode(body, lang, y, nameGen);
        if (handler.getTemporaryArraySize() != 0 || handler.getTemporarySparseArraySize() != 0)
            throw std::runtime_error(symbol + ": unexpected temporary arrays in the generated code");

        Generated g;
        g.temporaries = handler.getTemporaryVariableCount();
        g.tapeVariables = fun.size_var();
        std::ostringstream fn;
        fn << "void " << symbol << "(const double* x, double* y)\n{\n";
        if (g.temporaries > 0) fn << "   double v[" << g.temporaries << "];\n";
        fn << body.str() << "}\n\n";
        g.source = fn.str();
        return g;
    }

    void define(std::ostream& h, const std::string& macro, const std::string& value)
    {
        h << "#define " << macro << " " << value << "\n";
    }
    void define(std::ostream& h, const std::string& macro, long long value) { define(h, macro, std::to_string(value)); }

    /// Dimension, block offsets, layouts and hash: everything that is not a
    /// function. All of it is read from the configuration and EkfMaps.h.
    void writeConfigurationMacros(
        std::ostream& h, const Options& o, const Est::Configuration& cfg, const std::string& P)
    {
        h << "/* Configuration descriptor (the hash input):\n";
        std::istringstream descriptor(cfg.descriptor(o.convention));
        for (std::string line; std::getline(descriptor, line);) h << " *   " << line << "\n";
        h << " */\n";
        define(h, P + "CONFIG_NAME", "\"" + o.configName + "\"");
        define(h, P + "CONFIG_HASH", hashString(cfg.hash(o.convention)) + "ULL");
        define(h, P + "CONFIG_HASH_STRING", "\"" + hashString(cfg.hash(o.convention)) + "\"");
        define(h, P + "ERROR_CONVENTION", "\"" + std::string(Est::toString(o.convention)) + "\"");
        define(h, P + "ERROR_CONVENTION_RIGHT_INVARIANT", o.convention == Est::ErrorConvention::RightInvariant ? 1 : 0);
        define(h, P + "ERROR_DIM", cfg.dimension());
        define(h, P + "NOISE_DIM", Est::noiseDimension(cfg));

        h << "\n/* Error-state blocks, in canonical order: offset and dimension in the error vector. */\n";
        for (const Est::BlockSpec& s : Est::kBlockLibrary) {
            if (!cfg.has(s.id)) continue;
            define(h, P + "BLOCK_" + upper(s.name) + "_OFFSET", cfg.offset(s.id));
            define(h, P + "BLOCK_" + upper(s.name) + "_DIM", s.dim);
        }
        const int nav = cfg.offset(Est::BlockId::NavCore);
        h << "/* Navigation core: phi (rad), nu (m/s), rho (m), 3 each. */\n";
        define(h, P + "ERROR_ATTITUDE", nav + Est::NavCoreLayout::kAttitude);
        define(h, P + "ERROR_VELOCITY", nav + Est::NavCoreLayout::kVelocity);
        define(h, P + "ERROR_POSITION", nav + Est::NavCoreLayout::kPosition);

        h << "\n/* Process-noise vector (qc_diag is the diagonal of its PSD matrix). */\n";
        define(h, P + "NOISE_GYRO_OFFSET", 0);
        define(h, P + "NOISE_ACCEL_OFFSET", 3);
        for (const Est::BlockSpec& s : Est::kBlockLibrary)
            if (const int k = Est::noiseOffset(cfg, s.id); k >= 0)
                define(h, P + "NOISE_" + upper(s.name) + "_WALK_OFFSET", k);

        h << "\n/* Packed nominal state. Matrices are row-major everywhere. */\n";
        define(h, P + "STATE_DIM", CG::StateLayout::kDim);
        define(h, P + "STATE_ATTITUDE", CG::StateLayout::kAttitude);
        define(h, P + "STATE_VELOCITY", CG::StateLayout::kVelocity);
        define(h, P + "STATE_POSITION", CG::StateLayout::kPosition);
        define(h, P + "STATE_GYRO_BIAS", CG::StateLayout::kGyroBias);
        define(h, P + "STATE_ACCEL_BIAS", CG::StateLayout::kAccelBias);
        define(h, P + "STATE_BARO_BIAS", CG::StateLayout::kBaroBias);

        h << "\n/* Packed LCI frame: ECI position of the origin (3), rotation LCI -> ECI (3 x 3). */\n";
        define(h, P + "FRAME_DIM", CG::FrameLayout::kDim);
        define(h, P + "FRAME_ORIGIN", CG::FrameLayout::kOrigin);
        define(h, P + "FRAME_ROTATION", CG::FrameLayout::kRotation);
    }

    void writeFunctionMacros(
        std::ostream& h, const Est::Configuration& cfg, CG::Function f, const std::string& P, const Generated& g)
    {
        const std::string F = P + upper(CG::name(f)) + "_";
        const std::vector<CG::Segment> in = CG::inputSegments(cfg, f);
        const std::vector<CG::Segment> out = CG::outputSegments(cfg, f);
        h << "\n";
        define(h, F + "N_IN", CG::totalSize(in));
        define(h, F + "N_OUT", CG::totalSize(out));
        define(h, F + "N_TMP", static_cast<long long>(g.temporaries));
        int offset = 0;
        for (const CG::Segment& s : in) {
            define(h, F + "IN_" + upper(s.name), offset);
            offset += s.size;
        }
        offset = 0;
        for (const CG::Segment& s : out) {
            define(h, F + "OUT_" + upper(s.name), offset);
            offset += s.size;
        }
    }

    void run(const Options& o)
    {
        const Est::Configuration& cfg = configuration(o.configName);
        if (!cfg.valid()) throw std::runtime_error("configuration is not valid");
        const std::string P = upper(o.stem) + "_";
        const std::string comment = fileComment(o, cfg);

        std::ostringstream c;
        c << comment << "\n#include <math.h>\n\n#include \"" << o.stem << ".h\"\n\n";
        std::ostringstream h;
        h << comment << "\n#ifndef " << P << "H\n#define " << P << "H\n\n";
        writeConfigurationMacros(h, o, cfg, P);

        std::ostringstream prototypes;
        for (const CG::Function f : CG::kFunctions) {
            if (!CG::carried(cfg, f)) continue;
            const std::string symbol = o.stem + "_" + std::string(CG::name(f));
            const Generated g = generate(o, cfg, f, symbol);
            c << g.source;
            writeFunctionMacros(h, cfg, f, P, g);
            prototypes << "void " << symbol << "(const double* x, double* y);\n";
            std::cout << symbol << ": " << CG::totalSize(CG::inputSegments(cfg, f)) << " inputs, "
                      << CG::totalSize(CG::outputSegments(cfg, f)) << " outputs, tape " << g.tapeVariables
                      << " variables, " << g.temporaries << " temporaries, " << g.source.size() << " bytes of C\n";
        }
        h << "\n/* Each function reads x[0 .. N_IN) and writes y[0 .. N_OUT); x and y must not overlap.\n"
          << " * N_TMP doubles of automatic storage are used by the function. */\n"
          << "#ifdef __cplusplus\nextern \"C\" {\n#endif\n\n"
          << prototypes.str() << "\n#ifdef __cplusplus\n}\n#endif\n\n#endif /* " << P << "H */\n";

        std::filesystem::create_directories(o.outDir);
        const std::filesystem::path cFile = o.outDir / (o.stem + ".c");
        const std::filesystem::path hFile = o.outDir / (o.stem + ".h");
        std::ofstream(cFile, std::ios::binary) << c.str();
        std::ofstream(hFile, std::ios::binary) << h.str();
        std::cout << o.configName << ", " << Est::toString(o.convention) << ", hash "
                  << hashString(cfg.hash(o.convention)) << ": " << cFile.string() << " (" << c.str().size()
                  << " bytes), " << hFile.string() << " (" << h.str().size() << " bytes)\n";
    }
} // namespace

int main(int argc, char** argv)
{
    try {
        run(parse(argc, argv));
        return 0;
    } catch (const std::invalid_argument& e) {
        std::cerr << "generate_ekf_c: " << e.what() << "\n"
                  << kUsage;
        return 2;
    } catch (const std::exception& e) {
        std::cerr << "generate_ekf_c: " << e.what() << "\n";
        return 1;
    }
}
