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
#include <limits>
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
    double d{};
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

template<class S> static S evalNode(const pugi::xml_node& node,
                                    const std::unordered_map<std::string, S>& vars);

// Reduce an AD-typed value to its plain-double value for branching decisions
// (index selection, piecewise conditions, comparisons). Non-differentiable by
// design — only the *value* being carried forward stays in S.
template<class S>
static double toDouble(const S& v)
{
    if constexpr (std::is_same_v<S, double>) return v;
    else return CppAD::Value(CppAD::Var2Par(v));
}

// ── Arithmetic: times / plus / minus / divide / power ──────────────────────
template<class S>
static S evalArithmeticOp(const std::string& opName,
                          const std::vector<pugi::xml_node>& operands,
                          const std::unordered_map<std::string, S>& vars)
{
    if (opName == "times") {
        S r = S(1.0);
        for (auto& n : operands) r *= evalNode<S>(n, vars);
        return r;
    }
    if (opName == "plus") {
        S r = S(0.0);
        for (auto& n : operands) r += evalNode<S>(n, vars);
        return r;
    }
    if (opName == "minus" && operands.size() == 1)
        return -evalNode<S>(operands[0], vars);
    if (opName == "minus" && operands.size() == 2)
        return evalNode<S>(operands[0], vars) - evalNode<S>(operands[1], vars);
    if (opName == "divide" && operands.size() == 2)
        return evalNode<S>(operands[0], vars) / evalNode<S>(operands[1], vars);
    if (opName == "power" && operands.size() == 2) {
        using std::pow;
        return pow(evalNode<S>(operands[0], vars), evalNode<S>(operands[1], vars));
    }
    throw std::runtime_error(
        "DAVEMLAeroModel MathML: unsupported operator <" + opName + ">");
}

// ── Unary math functions: abs / cos / sin / tan / sqrt ──────────────────────
template<class S>
static S evalUnaryMathOp(const std::string& opName,
                         const std::vector<pugi::xml_node>& operands,
                         const std::unordered_map<std::string, S>& vars)
{
    S x = evalNode<S>(operands[0], vars);
    using std::abs; using std::cos; using std::sin; using std::tan; using std::sqrt;
    if (opName == "abs")  return abs(x);
    if (opName == "cos")  return cos(x);
    if (opName == "sin")  return sin(x);
    if (opName == "tan")  return tan(x);
    return sqrt(x);  // opName == "sqrt"
}

static bool isUnaryMathOp(const std::string& opName)
{
    return opName == "abs" || opName == "cos" || opName == "sin"
        || opName == "tan" || opName == "sqrt";
}

// ── csymbol — DAVE-ML named functions (e.g. atan2) ──────────────────────────
// Syntax: <apply><csymbol ...>atan2</csymbol><ci>y</ci><ci>x</ci></apply>
template<class S>
static S evalCsymbolOp(const pugi::xml_node& op,
                       const std::vector<pugi::xml_node>& operands,
                       const std::unordered_map<std::string, S>& vars)
{
    const std::string fnName = trimWS(op.child_value());
    if (fnName == "atan2" && operands.size() == 2) {
        using std::atan2;
        return atan2(evalNode<S>(operands[0], vars), evalNode<S>(operands[1], vars));
    }
    throw std::runtime_error(
        "DAVEMLAeroModel MathML: unsupported csymbol '" + fnName + "'");
}

// ── Comparisons (return S(1) for true, S(0) for false) ──────────────────────
// Used as piecewise conditions; branching itself is resolved in plain double.
template<class S>
static S evalComparisonOp(const std::string& opName,
                          const std::vector<pugi::xml_node>& operands,
                          const std::unordered_map<std::string, S>& vars)
{
    double lhsD = toDouble(evalNode<S>(operands[0], vars));
    double rhsD = toDouble(evalNode<S>(operands[1], vars));
    bool cond = false;
    if      (opName == "lt")  cond = lhsD <  rhsD;
    else if (opName == "gt")  cond = lhsD >  rhsD;
    else if (opName == "leq") cond = lhsD <= rhsD;
    else if (opName == "geq") cond = lhsD >= rhsD;
    else                      cond = lhsD == rhsD;  // "eq"
    return cond ? S(1.0) : S(0.0);
}

static bool isComparisonOp(const std::string& opName)
{
    return opName == "lt" || opName == "gt" || opName == "leq"
        || opName == "geq" || opName == "eq";
}

template<class S>
static S evalApplyNode(const pugi::xml_node& node,
                       const std::unordered_map<std::string, S>& vars)
{
    auto op = node.first_child();
    std::string opName = op.name();

    // Collect operand subtrees (skip operator element)
    std::vector<pugi::xml_node> operands;
    for (auto c = op.next_sibling(); c; c = c.next_sibling())
        operands.push_back(c);

    if (opName == "times" || opName == "plus" || opName == "minus"
        || opName == "divide" || opName == "power")
        return evalArithmeticOp<S>(opName, operands, vars);

    if (isUnaryMathOp(opName) && operands.size() == 1)
        return evalUnaryMathOp<S>(opName, operands, vars);

    if (opName == "csymbol")
        return evalCsymbolOp<S>(op, operands, vars);

    if (isComparisonOp(opName) && operands.size() == 2)
        return evalComparisonOp<S>(opName, operands, vars);

    // <apply><piecewise>...</piecewise></apply> — piecewise is the whole apply
    if (opName == "piecewise")
        return evalNode<S>(op, vars);

    throw std::runtime_error(
        "DAVEMLAeroModel MathML: unsupported operator <" + opName + ">");
}

template<class S>
static S evalPiecewiseNode(const pugi::xml_node& node,
                           const std::unordered_map<std::string, S>& vars)
{
    S result = S(0.0);
    bool matched = false;
    for (auto child = node.first_child(); child; child = child.next_sibling()) {
        std::string cn = child.name();
        if (cn == "piece") {
            auto val  = child.first_child();            // expression
            auto cond = val.next_sibling();              // condition
            bool condTrue = toDouble(evalNode<S>(cond, vars)) != 0.0;
            if (condTrue && !matched) {
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

template<class S>
static S evalNode(const pugi::xml_node& node,
                  const std::unordered_map<std::string, S>& vars)
{
    std::string tag = node.name();

    if (tag == "math")      return evalNode<S>(node.first_child(), vars);
    if (tag == "cn")        return S(std::stod(node.child_value()));
    if (tag == "apply")     return evalApplyNode<S>(node, vars);
    if (tag == "piecewise") return evalPiecewiseNode<S>(node, vars);

    if (tag == "ci") {
        const std::string id = trimWS(node.child_value());
        auto it = vars.find(id);
        if (it == vars.end())
            throw std::runtime_error("DAVEMLAeroModel MathML: unknown var '" + id + "'");
        return it->second;
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
        double xd{};
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
    std::size_t nvert = 1U << ndim;
    for (std::size_t v = 0; v < nvert; ++v)
    {
        // Linear index into the data table for this vertex
        std::size_t linearIdx = 0;
        S weight = S(1.0);
        std::size_t stride = 1;
        for (int d = (int)ndim - 1; d >= 0; --d) {
            std::size_t corner = (v >> d) & 1U;
            linearIdx += (idx[d] + corner) * stride;
            weight *= corner ? wt[d] : (S(1.0) - wt[d]);
            stride *= dims[d];
        }
        result += weight * S(data[linearIdx]);
    }
    return result;
}

// =============================================================================
// DAVEMLAeroModel — construction helpers
// =============================================================================

namespace {

// Per-variable metadata collected while walking the DAVE-ML XML tree, prior
// to topological sorting and EvalStep construction.
struct VarInfo {
    bool isInput{ false };
    bool isConst{ false };
    double constVal{ 0.0 };
    std::string mathmlXml;   // non-empty if calculated
    std::set<std::string> deps;
    bool   hasMinMax{ false };
    double minVal{ -std::numeric_limits<double>::infinity() };
    double maxVal{  std::numeric_limits<double>::infinity() };
};
using VarInfoMap = std::unordered_map<std::string, VarInfo>;

} // namespace

// Top-level griddedTableDef by gtID (resolves <griddedTableRef gtID="...">).
static std::unordered_map<std::string, pugi::xml_node>
collectTopLevelTableDefs(const pugi::xml_document& doc)
{
    std::unordered_map<std::string, pugi::xml_node> defs;
    for (const auto& xn : doc.select_nodes("/DAVEfunc/griddedTableDef"))
        defs[xn.node().attribute("gtID").as_string()] = xn.node();
    return defs;
}

static void parseBreakpointDefs(
    const pugi::xml_document& doc,
    std::unordered_map<std::string, DAVEMLAeroModel::BpVec>& bps)
{
    for (const auto& xn : doc.select_nodes("//breakpointDef")) {
        auto node = xn.node();
        std::string bpID = node.attribute("bpID").as_string();
        if (bpID.empty()) continue;
        auto bpVals = node.child("bpVals");
        if (bpVals)
            bps[bpID] = parseDoubles(bpVals.child_value());
    }
}

static void collectCiDeps(const pugi::xml_node& mathNode, std::set<std::string>& deps)
{
    for (const auto& ci : mathNode.select_nodes(".//ci"))
        deps.insert(trimWS(ci.node().child_value()));
}

// Collect variableDef entries (input/const/calculated) and their MathML
// dependency graph; also captures the sref/cbar/bspan geometry constants.
static void parseVariableDefs(const pugi::xml_document& doc, VarInfoMap& info,
                              double& srefFt2, double& cbarFt, double& bspanFt)
{
    for (const auto& xn : doc.select_nodes("//variableDef")) {
        auto node = xn.node();
        std::string varID = node.attribute("varID").as_string();
        if (varID.empty()) continue;
        VarInfo vi;

        if (node.child("isInput")) vi.isInput = true;

        auto iv = node.attribute("initialValue");
        if (iv) { vi.isConst = true; vi.constVal = iv.as_double(); }

        auto minA = node.attribute("minValue");
        auto maxA = node.attribute("maxValue");
        if (minA || maxA) {
            vi.hasMinMax = true;
            if (minA) vi.minVal = minA.as_double();
            if (maxA) vi.maxVal = maxA.as_double();
        }

        // Reference geometry constants
        if (varID == "sref")  srefFt2 = vi.constVal;
        if (varID == "cbar")  cbarFt  = vi.constVal;
        if (varID == "bspan") bspanFt = vi.constVal;

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
}

// Parse <function>/<griddedTableDef> (inline or via <griddedTableRef>) into
// gridded lookup tables, and register each table's output as a variableDef
// depending on its input vars.
static void parseFunctionTables(
    const pugi::xml_document& doc,
    const std::unordered_map<std::string, pugi::xml_node>& topLevelTableDefs,
    const std::unordered_map<std::string, DAVEMLAeroModel::BpVec>& bps,
    std::unordered_map<std::string, DAVEMLAeroModel::GridTable>& tables,
    VarInfoMap& info)
{
    for (const auto& xn : doc.select_nodes("//function")) {
        auto fnode = xn.node();

        std::vector<std::string> inputVarIDs;
        for (const auto& ivr : fnode.select_nodes("independentVarRef"))
            inputVarIDs.emplace_back(ivr.node().attribute("varID").as_string());

        auto dvr = fnode.child("dependentVarRef");
        if (!dvr) continue;
        std::string depVarID = dvr.attribute("varID").as_string();

        // Gridded table definition — inline or referenced
        auto gdef = fnode.select_node(".//griddedTableDef").node();
        if (!gdef) {
            auto ref = fnode.select_node(".//griddedTableRef").node();
            if (!ref) continue;
            auto it = topLevelTableDefs.find(ref.attribute("gtID").as_string());
            if (it == topLevelTableDefs.end()) continue;
            gdef = it->second;
        }

        DAVEMLAeroModel::GridTable gt;
        gt.inputVarIDs = inputVarIDs;

        for (const auto& bpref : gdef.select_nodes("breakpointRefs/bpRef"))
            gt.bpIDs.emplace_back(bpref.node().attribute("bpID").as_string());

        auto dtNode = gdef.child("dataTable");
        if (dtNode) gt.data = parseDoubles(dtNode.child_value());

        // Compute dims from breakpoints
        for (const auto& bpID : gt.bpIDs) {
            auto it = bps.find(bpID);
            gt.dims.push_back(it != bps.end() ? it->second.size() : 1);
        }

        tables[depVarID] = std::move(gt);

        // Register this var in info if not already there, with deps on inputs
        if (!info.contains(depVarID)) info[depVarID] = {};
        for (const auto& iv : inputVarIDs)
            info[depVarID].deps.insert(iv);
    }
}

// Topological sort of the variable dependency graph (Kahn's algorithm).
// Any remaining nodes after a cycle or unreachable state are appended in
// arbitrary order so construction never fails outright on malformed input.
static std::vector<std::string> topoSortVars(const VarInfoMap& info)
{
    std::unordered_map<std::string, int> inDeg;
    for (const auto& [id, vi] : info) {
        if (!inDeg.contains(id)) inDeg[id] = 0;
        for (const auto& dep : vi.deps)
            if (info.contains(dep)) inDeg[dep]; // ensure dep exists
    }
    // Build reverse adj: dep → set of vars that need dep
    std::unordered_map<std::string, std::vector<std::string>> rdeps;
    for (const auto& [id, vi] : info) {
        for (const auto& dep : vi.deps) {
            rdeps[dep].push_back(id);
            inDeg[id]; // ensure exists
        }
    }
    // Count real in-degrees
    for (auto& [id, vi] : info) inDeg[id] = 0;
    for (const auto& [id, vi] : info)
        for (const auto& dep : vi.deps)
            if (info.contains(dep)) inDeg[id]++;

    std::queue<std::string> ready;
    for (const auto& [id, deg] : inDeg)
        if (deg == 0 && info.contains(id)) ready.push(id);

    std::vector<std::string> sorted;
    while (!ready.empty()) {
        auto id = ready.front(); ready.pop();
        sorted.push_back(id);
        for (const auto& rdep : rdeps[id]) {
            if (--inDeg[rdep] == 0) ready.push(rdep);
        }
    }
    // Any remaining (cycle or unreachable) — append in arbitrary order
    for (const auto& [id, vi] : info)
        if (inDeg.contains(id) && inDeg[id] > 0) sorted.push_back(id);

    return sorted;
}

// Build the topologically-sorted EvalStep list consumed by evaluate<S>().
static std::vector<DAVEMLAeroModel::EvalStep> buildEvalSteps(
    const std::vector<std::string>& sorted,
    const VarInfoMap& info,
    const std::unordered_map<std::string, DAVEMLAeroModel::GridTable>& tables)
{
    std::vector<DAVEMLAeroModel::EvalStep> steps;
    steps.reserve(sorted.size());
    for (const auto& id : sorted) {
        auto it = info.find(id);
        if (it == info.end()) continue;
        const auto& vi = it->second;
        DAVEMLAeroModel::EvalStep step;
        step.varID      = id;
        step.isInput    = vi.isInput;
        step.isConst    = vi.isConst;
        step.constVal   = vi.constVal;
        step.isTable    = tables.contains(id);
        step.mathmlXml  = vi.mathmlXml;
        step.hasMinMax  = vi.hasMinMax;
        step.minVal     = vi.minVal;
        step.maxVal     = vi.maxVal;
        steps.emplace_back(std::move(step));
    }
    return steps;
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

    auto topLevelTableDefs = collectTopLevelTableDefs(doc);
    parseBreakpointDefs(doc, m_bps);

    VarInfoMap info;
    parseVariableDefs(doc, info, m_srefFt2, m_cbarFt, m_bspanFt);
    parseFunctionTables(doc, topLevelTableDefs, m_bps, m_tables, info);

    auto sorted = topoSortVars(info);
    m_steps = buildEvalSteps(sorted, info, m_tables);
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
        if (step.isInput && vars.contains(id)) continue;  // keep DML-input values, recompute everything else

        // Table > MathML calc > const initialValue (table overrides default)
        if (step.isTable)                    vars[id] = interpolate<S>(m_tables.at(id), vars);
        else if (!step.mathmlXml.empty())    vars[id] = evalMathMLStr<S>(step.mathmlXml, vars);
        else if (step.isConst)               vars[id] = S(step.constVal);
        if (!vars.contains(id))              vars[id] = S(0.0);

        if (step.hasMinMax) {
            if constexpr (std::is_same_v<S, double>) {
                vars[id] = std::clamp(vars[id], step.minVal, step.maxVal);
            } else {
                auto& v = vars[id];
                v = CppAD::CondExpLt(v, S(step.minVal), S(step.minVal), v);
                v = CppAD::CondExpGt(v, S(step.maxVal), S(step.maxVal), v);
            }
        }
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
    for (const auto& id : fn.inputVarIDs) {
        auto it = vars.find(id);
        inputs.push_back(it != vars.end() ? it->second : S(0.0));
    }
    std::vector<const BpVec*> bps;
    for (const auto& bpID : fn.bpIDs) {
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
        if (step.isInput && vars.contains(id)) continue;  // keep DML-input values, recompute everything else
        // Table takes priority over initialValue (initialValue is just a default)
        if (step.isTable)                    vars[id] = interpolate<S>(m_tables.at(id), vars);
        else if (!step.mathmlXml.empty())    vars[id] = evalMathMLStr<S>(step.mathmlXml, vars);
        else if (step.isConst)               vars[id] = S(step.constVal);
        if (!vars.contains(id))              vars[id] = S(0.0);

        // Apply variableDef minValue / maxValue clamping
        if (step.hasMinMax) {
            if constexpr (std::is_same_v<S, double>) {
                vars[id] = std::clamp(vars[id], step.minVal, step.maxVal);
            } else {
                // AD-safe conditional clamping
                auto& v = vars[id];
                v = CppAD::CondExpLt(v, S(step.minVal), S(step.minVal), v);
                v = CppAD::CondExpGt(v, S(step.maxVal), S(step.maxVal), v);
            }
        }
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
