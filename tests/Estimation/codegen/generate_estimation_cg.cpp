// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// generate_estimation_cg: tapes every map of CodeGenModels.h with
// CppAD::AD<CppAD::cg::CG<double>> and writes plain C for the map and for its
// dense Jacobian, plus a header with a table of the generated functions.
// Build-time fixture of test_estimation_codegen (TODO-sensor-fusion.md,
// "Things to check": CppAD code-generation compatibility); the step-8
// generator tool is separate work.
//
//   generate_estimation_cg <output directory>
//
// writes <dir>/estimation_cg.c and <dir>/estimation_cg.h.
// ------------------------------------------------------------------------------

// CppADCodeGen must precede every CppAD include: it includes
// cppad/cppad.hpp itself, after declaring CG<Base> as a CppAD base type, and
// refuses to compile if cppad.hpp came first. CppADCodeGenCore.h stands in for
// <cppad/cg.hpp>, which MSVC cannot compile (see that header).
// cppad_eigen.hpp then defines Eigen's matrix plugin before Eigen/Core is
// first seen, and cppadcg_eigen.hpp adds Eigen's NumTraits for AD<CG<double>>.
#include "CppADCodeGenCore.h"
// clang-format off
#include <cppad/example/cppad_eigen.hpp>
#include <cppad/cg/support/cppadcg_eigen.hpp>
// clang-format on

#include <cstddef>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "CodeGenModels.h"

namespace {
    using CGD = CppAD::cg::CG<double>;
    using ADCG = CppAD::AD<CGD>;
    namespace CGT = Aetherion::Estimation::CodeGenTest;

    /// Source of one C function `void <name>(const double* x, double* y)`
    /// computing the dependents `dep` built on `handler`'s independents.
    std::string cFunction(const std::string& name, CppAD::cg::CodeHandler<double>& handler, std::vector<CGD>& dep)
    {
        CppAD::cg::LanguageC<double> lang("double");
        // digits10 (15), the default, rounds the frame's 6.4e6 m ECI origin and
        // the WGS-84 constants: 17 digits reproduce every double exactly.
        lang.setParameterPrecision(std::numeric_limits<double>::max_digits10);
        CppAD::cg::LangCDefaultVariableNameGenerator<double> nameGen("y", "x", "v");
        std::ostringstream body;
        handler.generateCode(body, lang, dep, nameGen);
        // LanguageC writes the statements only; the temporaries are declared
        // here. Arrays appear only with atomic functions or loops, neither of
        // which these maps use.
        if (handler.getTemporaryArraySize() != 0 || handler.getTemporarySparseArraySize() != 0)
            throw std::runtime_error(name + ": unexpected temporary arrays in the generated code");

        std::ostringstream fn;
        fn << "void " << name << "(const double* x, double* y)\n{\n";
        if (handler.getTemporaryVariableCount() > 0)
            fn << "   double v[" << handler.getTemporaryVariableCount() << "];\n";
        fn << body.str() << "}\n\n";
        return fn.str();
    }

    /// Tape model m at the inputs of point A and generate its value and
    /// Jacobian functions.
    std::string generate(const CGT::ModelSpec& m, std::ostream& log)
    {
        const std::vector<double> x0 = CGT::inputs(m, CGT::pointA());
        const auto nIn = static_cast<std::size_t>(m.nIn);

        std::vector<ADCG> ax(nIn);
        for (std::size_t i = 0; i < nIn; ++i)
            ax[i] = ADCG(CGD(x0[i]));
        CppAD::Independent(ax);
        const std::vector<ADCG> ay = CGT::evaluate<ADCG>(m, CGT::castFrame<ADCG>(CGT::testFrame()), ax);
        CppAD::ADFun<CGD> fun(ax, ay);

        const std::string stem = "aeth_cg_" + std::string(m.name);
        std::string src;
        {
            CppAD::cg::CodeHandler<double> handler;
            std::vector<CGD> x(nIn);
            handler.makeVariables(x);
            std::vector<CGD> y = fun.Forward(0, x);
            src += cFunction(stem, handler, y);
        }
        {
            CppAD::cg::CodeHandler<double> handler;
            std::vector<CGD> x(nIn);
            handler.makeVariables(x);
            std::vector<CGD> jac = fun.Jacobian(x);
            src += cFunction(stem + "_jacobian", handler, jac);
        }
        log << m.name << ": " << m.nIn << " inputs, " << m.nOut << " outputs, tape " << fun.size_var()
            << " variables, " << src.size() << " bytes of C\n";
        return src;
    }
} // namespace

int main(int argc, char** argv)
{
    if (argc != 2) {
        std::cerr << "usage: generate_estimation_cg <output directory>\n";
        return 2;
    }
    try {
        const std::filesystem::path dir(argv[1]);
        std::filesystem::create_directories(dir);

        std::ostringstream c;
        c << "/* Generated by generate_estimation_cg (CppADCodeGen 2.5.0). Do not edit. */\n"
          << "#include <math.h>\n#include \"estimation_cg.h\"\n\n";
        std::ostringstream h;
        h << "/* Generated by generate_estimation_cg. Do not edit. */\n#pragma once\n\n#ifdef __cplusplus\nextern "
             "\"C\" {\n#endif\n\n"
          << "typedef void (*aeth_cg_fn)(const double* x, double* y);\n"
          << "typedef struct\n{\n    const char* name;\n    int n_in;\n    int n_out;\n    aeth_cg_fn value;\n"
          << "    aeth_cg_fn jacobian; /* row-major, n_out x n_in */\n} aeth_cg_model;\n\n";

        for (const CGT::ModelSpec& m : CGT::kModels) {
            const std::string stem = "aeth_cg_" + std::string(m.name);
            c << generate(m, std::cout);
            h << "void " << stem << "(const double* x, double* y);\nvoid " << stem
              << "_jacobian(const double* x, double* y);\n";
        }
        h << "\n#define AETH_CG_MODEL_COUNT " << CGT::kModels.size()
          << "\nextern const aeth_cg_model aeth_cg_models[AETH_CG_MODEL_COUNT];\n\n#ifdef __cplusplus\n}\n#endif\n";

        c << "const aeth_cg_model aeth_cg_models[AETH_CG_MODEL_COUNT] = {\n";
        for (const CGT::ModelSpec& m : CGT::kModels) {
            const std::string stem = "aeth_cg_" + std::string(m.name);
            c << "    { \"" << m.name << "\", " << m.nIn << ", " << m.nOut << ", " << stem << ", " << stem
              << "_jacobian },\n";
        }
        c << "};\n";

        std::ofstream(dir / "estimation_cg.c", std::ios::binary) << c.str();
        std::ofstream(dir / "estimation_cg.h", std::ios::binary) << h.str();
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "generate_estimation_cg: " << e.what() << "\n";
        return 1;
    }
}
