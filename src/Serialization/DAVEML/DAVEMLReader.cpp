// ------------------------------------------------------------------------------
// Project: Aetherion
// Copyright(c) 2025-2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: MIT
// License-Filename: LICENSE
// ------------------------------------------------------------------------------

#include <Aetherion/Serialization/DAVEML/DAVEMLReader.h>

// pugixml (vendored via Janus ThirdParty, MIT licence)
#include <pugixml.hpp>

#include <cmath>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace Aetherion::Serialization {

// ─────────────────────────────────────────────────────────────────────────────
// Unit conversion factor to SI
// ─────────────────────────────────────────────────────────────────────────────

static double unitFactor(const std::string& u)
{
    if (u == "slugft2" || u == "slug*ft^2" || u == "slug-ft2")
        return 1.355817948329279;
    if (u == "slug")
        return 14.593902937206364;
    if (u == "ft")
        return 0.3048;
    if (u == "pct")
        return 0.01;
    return 1.0;
}

// ─────────────────────────────────────────────────────────────────────────────
// Minimal MathML arithmetic evaluator (recursive)
// ─────────────────────────────────────────────────────────────────────────────

using VarEntry = DAVEMLReader::VarEntry;
using VarMap   = std::unordered_map<std::string, VarEntry>;

static std::string trimWS(const std::string& s)
{
    auto b = s.find_first_not_of(" \t\n\r");
    if (b == std::string::npos) return {};
    auto e = s.find_last_not_of(" \t\n\r");
    return s.substr(b, e - b + 1);
}

static double evalNode(const pugi::xml_node& node, const VarMap& vars)
{
    std::string tag = node.name();

    if (tag == "cn")
        return std::stod(node.child_value());

    if (tag == "ci")
    {
        const std::string id = trimWS(node.child_value());
        auto it = vars.find(id);
        if (it == vars.end())
            throw std::runtime_error(
                "DAVEMLReader MathML: unknown varID '" + id + "'");
        return it->second.value;
    }

    if (tag == "math")
        return evalNode(node.first_child(), vars);

    if (tag == "apply")
    {
        const auto op    = node.first_child();
        const std::string opName = op.name();

        std::vector<double> args;
        for (auto c = op.next_sibling(); c; c = c.next_sibling())
            args.push_back(evalNode(c, vars));

        if (opName == "times")  { double r = 1.0; for (double v : args) r *= v; return r; }
        if (opName == "plus")   { double r = 0.0; for (double v : args) r += v; return r; }
        if (opName == "minus")
        {
            if (args.size() == 1) return -args[0];
            if (args.size() == 2) return args[0] - args[1];
        }
        if (opName == "divide" && args.size() == 2) return args[0] / args[1];
        if (opName == "power"  && args.size() == 2) return std::pow(args[0], args[1]);

        throw std::runtime_error(
            "DAVEMLReader MathML: unsupported operator <" + opName + ">");
    }

    throw std::runtime_error(
        "DAVEMLReader MathML: unexpected element <" + tag + ">");
}

// ─────────────────────────────────────────────────────────────────────────────
// Helper: serialise a pugixml node subtree to string
// ─────────────────────────────────────────────────────────────────────────────

static std::string nodeToStr(const pugi::xml_node& n)
{
    std::ostringstream oss;
    n.print(oss);
    return oss.str();
}

// ─────────────────────────────────────────────────────────────────────────────
// Re-evaluate all calculated entries using the current var map
// ─────────────────────────────────────────────────────────────────────────────

static void reeval(VarMap& vars)
{
    for (auto& [id, entry] : vars)
    {
        if (!entry.isCalculated || entry.mathmlXml.empty()) continue;
        pugi::xml_document d;
        d.load_string(entry.mathmlXml.c_str());
        entry.value = evalNode(d.first_child(), vars);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// DAVEMLReader — constructor
// ─────────────────────────────────────────────────────────────────────────────

DAVEMLReader::DAVEMLReader(const std::string& path)
{
    pugi::xml_document doc;
    auto res = doc.load_file(path.c_str());
    if (!res)
        throw std::runtime_error(
            "DAVEMLReader: cannot open '" + path + "': " + res.description());

    for (auto& xn : doc.select_nodes("//variableDef"))
    {
        auto node  = xn.node();
        std::string varID = node.attribute("varID").as_string();
        if (varID.empty()) continue;

        VarEntry e;
        e.units = node.attribute("units").as_string();

        auto iv = node.attribute("initialValue");
        if (iv) e.value = iv.as_double();

        auto calc = node.child("calculation");
        if (calc)
        {
            e.isCalculated = true;
            auto math = calc.child("math");
            if (math) e.mathmlXml = nodeToStr(math);
        }

        m_vars[varID] = std::move(e);
    }

    // Initial evaluation pass for calculated outputs
    reeval(m_vars);
}

// ─────────────────────────────────────────────────────────────────────────────
// Public API
// ─────────────────────────────────────────────────────────────────────────────

double DAVEMLReader::getValue(const std::string& varID) const
{
    auto it = m_vars.find(varID);
    if (it == m_vars.end())
        throw std::out_of_range("DAVEMLReader: varID '" + varID + "' not found");
    return it->second.value;
}

double DAVEMLReader::getValueSI(const std::string& varID) const
{
    auto it = m_vars.find(varID);
    if (it == m_vars.end())
        throw std::out_of_range("DAVEMLReader: varID '" + varID + "' not found");
    return it->second.value * unitFactor(it->second.units);
}

void DAVEMLReader::setInput(const std::string& varID, double value)
{
    auto it = m_vars.find(varID);
    if (it == m_vars.end()) return;
    it->second.value = value;
    reeval(m_vars);
}

bool DAVEMLReader::hasVar(const std::string& varID) const
{
    return m_vars.count(varID) > 0;
}

double DAVEMLReader::evalMathML(const std::string& xml) const
{
    pugi::xml_document d;
    d.load_string(xml.c_str());
    return evalNode(d.first_child(), m_vars);
}

} // namespace Aetherion::Serialization
