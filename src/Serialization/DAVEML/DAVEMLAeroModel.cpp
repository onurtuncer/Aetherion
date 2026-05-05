// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------

#include <Aetherion/Serialization/DAVEML/DAVEMLAeroModel.h>

#include <pugixml.hpp>
#include <cppad/cppad.hpp>   // for CppAD::Value, CppAD::CondExpLt

#include <algorithm>
#include <cmath>
#include <queue>
#include <set>
#include <sstream>
#include <stdexcept>
#include <unordered_set>

namespace Aetherion::Serialization {

// =============================================================================
// XML helpers
// =============================================================================

static std::string nodeToStr(const pugi::xml_node& n)
{
    std::ostringstream oss;
    n.print(oss);
    return oss.str();
}

static std::string trimWS(const std::string& s)
{
    auto b = s.find_first_not_of(" \t\n\r");
    if (b == std::string::npos) return {};
    return s.substr(b, s.find_last_not_of(" \t\n\r") - b + 1);
}

// Parse comma/whitespace separated doubles (for breakpoints and data tables)
static std::vector<double> parseDoubles(const std::string& text)
{
    std::vector<double> v;
    std::istringstream iss(text);
    double d;
    while (iss >> d) {
        v.push_back(d);
        char c = 0;
        while (iss.get(c) && (c == ',' || c == ' ' || c == '\n' || c == '\r' || c == '\t'));
        if (iss) iss.putback(c);
    }
    return v;
}

// =============================================================================
// MathML node evaluator (templated, AD-safe)
// =============================================================================

template<class S>
static S evalNode(const pugi::xml_node& node,
                  const std::unordered_map<std::string, S>& vars)
{
    std::string tag = node.name();

    if (tag == "math")   return evalNode<S>(node.first_child(), vars);
    if (tag == "cn")     return S(std::stod(node.child_value()));
    if (tag == "ci") {
        const std::string id = trimWS(node.child_value());
        auto it = vars.find(id);
        if (it == vars.end())
            throw std::runtime_error("DAVEMLAeroModel MathML: unknown var '" + id + "'");
        return it->second;
    }

    if (tag == "apply") {
        auto op = node.first_child();
        std::string opName = op.name();

        // Collect operand subtrees (skip operator element)
        std::vector<pugi::xml_node> operandNodes;
        for (auto c = op.next_sibling(); c; c = c.next_sibling())
            operandNodes.push_back(c);

        // ── Arithmetic ─────────────────────────────────────────────────────
        if (opName == "times") {
            S r = S(1.0);
            for (auto& n : operandNodes) r *= evalNode<S>(n, vars);
            return r;
        }
        if (opName == "plus") {
            S r = S(0.0);
            for (auto& n : operandNodes) r += evalNode<S>(n, vars);
            return r;
        }
        if (opName == "minus") {
            if (operandNodes.size() == 1) return -evalNode<S>(operandNodes[0], vars);
            if (operandNodes.size() == 2)
                return evalNode<S>(operandNodes[0], vars)
                     - evalNode<S>(operandNodes[1], vars);
        }
        if (opName == "divide" && operandNodes.size() == 2)
            return evalNode<S>(operandNodes[0], vars)
                 / evalNode<S>(operandNodes[1], vars);
        if (opName == "power" && operandNodes.size() == 2) {
            using std::pow;
            return pow(evalNode<S>(operandNodes[0], vars),
                       evalNode<S>(operandNodes[1], vars));
        }
        // ── Unary abs ──────────────────────────────────────────────────────
        if (opName == "abs" && operandNodes.size() == 1) {
            using std::abs;
            return abs(evalNode<S>(operandNodes[0], vars));
        }
        // ── Comparisons (return S(1) for true, S(0) for false) ─────────────
        // Used as piecewise conditions; AD path uses CondExp.
        if ((opName == "lt" || opName == "gt" ||
             opName == "leq" || opName == "geq" || opName == "eq")
            && operandNodes.size() == 2)
        {
            S lhs = evalNode<S>(operandNodes[0], vars);
            S rhs = evalNode<S>(operandNodes[1], vars);
            // Evaluate condition using double (AD-safe: only affects branching,
            // not the value being differentiated).
            double lhsD, rhsD;
            if constexpr (std::is_same_v<S, double>) {
                lhsD = lhs; rhsD = rhs;
            } else {
                lhsD = CppAD::Value(CppAD::Var2Par(lhs));
                rhsD = CppAD::Value(CppAD::Var2Par(rhs));
            }
            bool cond = (opName == "lt")  ? lhsD <  rhsD :
                        (opName == "gt")  ? lhsD >  rhsD :
                        (opName == "leq") ? lhsD <= rhsD :
                        (opName == "geq") ? lhsD >= rhsD :
                                            lhsD == rhsD;
            return cond ? S(1.0) : S(0.0);
        }

        // <apply><piecewise>...</piecewise></apply> — piecewise is the whole apply
        if (opName == "piecewise")
            return evalNode<S>(op, vars);

        throw std::runtime_error(
            "DAVEMLAeroModel MathML: unsupported operator <" + opName + ">");
    }

    // ── Piecewise ──────────────────────────────────────────────────────────
    if (tag == "piecewise") {
        S result = S(0.0);
        bool matched = false;
        for (auto child = node.first_child(); child; child = child.next_sibling()) {
            std::string cn = child.name();
            if (cn == "piece") {
                auto val  = child.first_child();            // expression
                auto cond = val.next_sibling();             // condition
                S condVal = evalNode<S>(cond, vars);
                double condD;
                if constexpr (std::is_same_v<S, double>) condD = condVal;
                else condD = CppAD::Value(CppAD::Var2Par(condVal));
                if (condD != 0.0 && !matched) {
                    result = evalNode<S>(val, vars);
                    matched = true;
                }
            } else if (cn == "otherwise" && !matched) {
                result = evalNode<S>(child.first_child(), vars);
                matched = true;
            }
        }
        return result;
    }

    throw std::runtime_error(
        "DAVEMLAeroModel MathML: unexpected element <" + tag + ">");
}

// =============================================================================
// N-D linear interpolation (AD-safe)
// =============================================================================

template<class S>
static S ndInterp(const std::vector<double>& data,
                  const std::vector<std::size_t>& dims,
                  const std::vector<const std::vector<double>*>& bps,
                  const std::vector<S>& inputs)
{
    std::size_t ndim = dims.size();
    assert(inputs.size() == ndim);

    // For each dimension find the segment index (using double) and weight (S)
    std::vector<std::size_t> idx(ndim);
    std::vector<S>           wt(ndim);

    for (std::size_t d = 0; d < ndim; ++d) {
        const auto& bp = *bps[d];
        double xd;
        if constexpr (std::is_same_v<S, double>) xd = inputs[d];
        else xd = CppAD::Value(CppAD::Var2Par(inputs[d]));

        // Clamp to breakpoint range
        xd = std::max(bp.front(), std::min(bp.back(), xd));

        // Find segment
        std::size_t i = 0;
        while (i + 1 < bp.size() - 1 && bp[i + 1] <= xd) ++i;
        idx[d] = i;

        double span = bp[i + 1] - bp[i];
        // Use the original AD input for the weight so CppAD records the correct
        // derivative (d wt/d input = 1/span). The double xd is only used above
        // for index selection, which is non-differentiable and treated as constant.
        wt[d] = (span > 0.0) ? (inputs[d] - S(bp[i])) / S(span) : S(0.0);
    }

    // N-D bilinear interpolation: iterate over 2^ndim vertices
    S result = S(0.0);
    std::size_t nvert = 1u << ndim;
    for (std::size_t v = 0; v < nvert; ++v)
    {
        // Linear index into the data table for this vertex
        std::size_t linearIdx = 0;
        S weight = S(1.0);
        std::size_t stride = 1;
        for (int d = (int)ndim - 1; d >= 0; --d) {
            std::size_t corner = (v >> d) & 1u;
            linearIdx += (idx[d] + corner) * stride;
            weight *= corner ? wt[d] : (S(1.0) - wt[d]);
            stride *= dims[d];
        }
        result += weight * S(data[linearIdx]);
    }
    return result;
}

// =============================================================================
// DAVEMLAeroModel — construction
// =============================================================================

DAVEMLAeroModel::DAVEMLAeroModel(const std::string& path)
{
    pugi::xml_document doc;
    auto res = doc.load_file(path.c_str());
    if (!res)
        throw std::runtime_error(
            "DAVEMLAeroModel: cannot open '" + path + "': " + res.description());

    // ── Step 1a: parse top-level griddedTableDef by gtID (for griddedTableRef) ─
    std::unordered_map<std::string, pugi::xml_node> topLevelTableDefs;
    for (auto& xn : doc.select_nodes("/DAVEfunc/griddedTableDef"))
        topLevelTableDefs[xn.node().attribute("gtID").as_string()] = xn.node();

    // ── Step 1b: parse breakpointDef ────────────────────────────────────────
    for (auto& xn : doc.select_nodes("//breakpointDef")) {
        auto node = xn.node();
        std::string bpID = node.attribute("bpID").as_string();
        if (bpID.empty()) continue;
        auto bpVals = node.child("bpVals");
        if (bpVals)
            m_bps[bpID] = parseDoubles(bpVals.child_value());
    }

    // ── Step 2: collect variableDef entries and their deps ──────────────────
    // We store: varID → {isInput, isConst, constVal, mathmlXml}
    // We also build a dep graph: varID → set of varIDs it reads

    struct VarInfo {
        bool isInput{ false };
        bool isConst{ false };
        double constVal{ 0.0 };
        std::string mathmlXml;   // non-empty if calculated
        std::string tableOutputID;  // if this var is a function output
        std::set<std::string> deps;
    };
    std::unordered_map<std::string, VarInfo> info;

    auto collectCiDeps = [&](const pugi::xml_node& mathNode,
                              std::set<std::string>& deps) {
        for (auto& ci : mathNode.select_nodes(".//ci"))
            deps.insert(trimWS(ci.node().child_value()));
    };

    for (auto& xn : doc.select_nodes("//variableDef")) {
        auto node = xn.node();
        std::string varID = node.attribute("varID").as_string();
        if (varID.empty()) continue;
        VarInfo vi;

        if (node.child("isInput")) vi.isInput = true;

        auto iv = node.attribute("initialValue");
        if (iv) { vi.isConst = true; vi.constVal = iv.as_double(); }

        // Reference geometry constants
        if (varID == "sref") sref_ft2 = vi.constVal;
        if (varID == "cbar") cbar_ft  = vi.constVal;
        if (varID == "bspan") bspan_ft = vi.constVal;

        auto calc = node.child("calculation");
        if (calc) {
            auto math = calc.child("math");
            if (math) {
                vi.mathmlXml = nodeToStr(math);
                collectCiDeps(math, vi.deps);
            }
        }
        info[varID] = std::move(vi);
    }

    // ── Step 3: parse function tables ──────────────────────────────────────
    for (auto& xn : doc.select_nodes("//function")) {
        auto fnode = xn.node();

        // Collect input var IDs and their order
        std::vector<std::string> inputVarIDs;
        for (auto& ivr : fnode.select_nodes("independentVarRef"))
            inputVarIDs.push_back(ivr.node().attribute("varID").as_string());

        // Dependent var ID = the output of this function
        auto dvr = fnode.child("dependentVarRef");
        if (!dvr) continue;
        std::string depVarID = dvr.attribute("varID").as_string();

        // Gridded table definition — inline or referenced
        auto gdef = fnode.select_node(".//griddedTableDef").node();
        if (!gdef) {
            // Try <griddedTableRef gtID="..."> → resolve from top-level defs
            auto ref = fnode.select_node(".//griddedTableRef").node();
            if (!ref) continue;
            auto it = topLevelTableDefs.find(ref.attribute("gtID").as_string());
            if (it == topLevelTableDefs.end()) continue;
            gdef = it->second;
        }

        GridTable gt;
        gt.inputVarIDs = inputVarIDs;

        for (auto& bpref : gdef.select_nodes("breakpointRefs/bpRef"))
            gt.bpIDs.push_back(bpref.node().attribute("bpID").as_string());

        auto dtNode = gdef.child("dataTable");
        if (dtNode) gt.data = parseDoubles(dtNode.child_value());

        // Compute dims from breakpoints
        for (auto& bpID : gt.bpIDs) {
            auto it = m_bps.find(bpID);
            if (it != m_bps.end())
                gt.dims.push_back(it->second.size());
            else
                gt.dims.push_back(1);
        }

        m_tables[depVarID] = std::move(gt);

        // Register this var in info if not already there
        if (!info.count(depVarID)) info[depVarID] = {};
        // Add dependencies on input vars
        for (auto& iv : inputVarIDs)
            info[depVarID].deps.insert(iv);
    }

    // ── Step 4: topological sort (Kahn's algorithm) ─────────────────────────
    // in-degree count
    std::unordered_map<std::string, int> inDeg;
    for (auto& [id, vi] : info) {
        if (!inDeg.count(id)) inDeg[id] = 0;
        for (auto& dep : vi.deps)
            if (info.count(dep)) inDeg[dep]; // ensure dep exists
    }
    // Build reverse adj: dep → set of vars that need dep
    std::unordered_map<std::string, std::vector<std::string>> rdeps;
    for (auto& [id, vi] : info) {
        for (auto& dep : vi.deps) {
            rdeps[dep].push_back(id);
            inDeg[id]; // ensure exists
        }
    }
    // Count real in-degrees
    for (auto& [id, vi] : info) inDeg[id] = 0;
    for (auto& [id, vi] : info)
        for (auto& dep : vi.deps)
            if (info.count(dep)) inDeg[id]++;

    std::queue<std::string> ready;
    for (auto& [id, deg] : inDeg)
        if (deg == 0 && info.count(id)) ready.push(id);

    std::vector<std::string> sorted;
    while (!ready.empty()) {
        auto id = ready.front(); ready.pop();
        sorted.push_back(id);
        for (auto& rdep : rdeps[id]) {
            if (!--inDeg[rdep]) ready.push(rdep);
        }
    }
    // Any remaining (cycle or unreachable) — append in arbitrary order
    for (auto& [id, vi] : info)
        if (inDeg.count(id) && inDeg[id] > 0) sorted.push_back(id);

    // ── Step 5: build evaluation steps ─────────────────────────────────────
    for (auto& id : sorted) {
        auto it = info.find(id);
        if (it == info.end()) continue;
        const auto& vi = it->second;
        EvalStep step;
        step.varID = id;
        step.isInput = vi.isInput;
        step.isConst = vi.isConst;
        step.constVal = vi.constVal;
        step.isTable  = m_tables.count(id) > 0;
        step.mathmlXml = vi.mathmlXml;
        m_steps.push_back(step);
    }
}

// =============================================================================
// DAVEMLAeroModel::evaluate<S>
// =============================================================================

template<class S>
DAVEMLAeroModel::Outputs<S>
DAVEMLAeroModel::evaluate(const Inputs<S>& in) const
{
    std::unordered_map<std::string, S> vars;
    vars.reserve(m_steps.size() + 16);

    // Seed inputs (DAVE-ML standard variable names from the aero file)
    vars["vt"]    = in.vt_fps;
    vars["alpha"] = in.alpha_deg;
    vars["beta"]  = in.beta_deg;
    vars["p"]     = in.p_rps;
    vars["q"]     = in.q_rps;
    vars["r"]     = in.r_rps;
    vars["el"]    = in.el_deg;
    vars["ail"]   = in.ail_deg;
    vars["rdr"]   = in.rdr_deg;

    // Evaluate each step in topological order
    for (const auto& step : m_steps) {
        const auto& id = step.varID;
        if (vars.count(id)) continue;  // already set (e.g. an input)

        // Table > MathML calc > const initialValue (table overrides default)
        if (step.isTable)                    vars[id] = interpolate<S>(m_tables.at(id), vars);
        else if (!step.mathmlXml.empty())    vars[id] = evalMathMLStr<S>(step.mathmlXml, vars);
        else if (step.isConst)               vars[id] = S(step.constVal);
        if (!vars.count(id))                 vars[id] = S(0.0);
    }

    Outputs<S> out;
    auto get = [&](const char* id) -> S {
        auto it = vars.find(id);
        return it != vars.end() ? it->second : S(0.0);
    };
    out.cx = get("cx");
    out.cy = get("cy");
    out.cz = get("cz");
    out.cl = get("cl");
    out.cm = get("cm");
    out.cn = get("cn");
    return out;
}

// =============================================================================
// Helper implementations
// =============================================================================

template<class S>
S DAVEMLAeroModel::interpolate(
    const GridTable& fn,
    const std::unordered_map<std::string, S>& vars) const
{
    std::vector<S> inputs;
    inputs.reserve(fn.inputVarIDs.size());
    for (auto& id : fn.inputVarIDs) {
        auto it = vars.find(id);
        inputs.push_back(it != vars.end() ? it->second : S(0.0));
    }
    std::vector<const BpVec*> bps;
    for (auto& bpID : fn.bpIDs) {
        auto it = m_bps.find(bpID);
        if (it == m_bps.end())
            throw std::runtime_error(
                "DAVEMLAeroModel: breakpoint '" + bpID + "' not found");
        bps.push_back(&it->second);
    }
    return ndInterp<S>(fn.data, fn.dims, bps, inputs);
}

template<class S>
S DAVEMLAeroModel::evalMathMLStr(
    const std::string& xml,
    const std::unordered_map<std::string, S>& vars) const
{
    pugi::xml_document d;
    d.load_string(xml.c_str());
    return evalNode<S>(d.first_child(), vars);
}

// =============================================================================
// DAVEMLAeroModel::evaluateRaw<S>  — generic, map-based entry point
// =============================================================================

template<class S>
std::unordered_map<std::string, S>
DAVEMLAeroModel::evaluateRaw(std::unordered_map<std::string, S> vars) const
{
    for (const auto& step : m_steps) {
        const auto& id = step.varID;
        if (vars.count(id)) continue;
        // Table takes priority over initialValue (initialValue is just a default)
        if (step.isTable)                    vars[id] = interpolate<S>(m_tables.at(id), vars);
        else if (!step.mathmlXml.empty())    vars[id] = evalMathMLStr<S>(step.mathmlXml, vars);
        else if (step.isConst)               vars[id] = S(step.constVal);
        if (!vars.count(id))                 vars[id] = S(0.0);
    }
    return vars;
}

// Explicit template instantiations for double and AD<double>
template DAVEMLAeroModel::Outputs<double>
DAVEMLAeroModel::evaluate(const Inputs<double>&) const;

template DAVEMLAeroModel::Outputs<CppAD::AD<double>>
DAVEMLAeroModel::evaluate(const Inputs<CppAD::AD<double>>&) const;

template std::unordered_map<std::string, double>
DAVEMLAeroModel::evaluateRaw(std::unordered_map<std::string, double>) const;

template std::unordered_map<std::string, CppAD::AD<double>>
DAVEMLAeroModel::evaluateRaw(std::unordered_map<std::string, CppAD::AD<double>>) const;

} // namespace Aetherion::Serialization
