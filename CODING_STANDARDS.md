# Coding Standards

Aetherion adopts a practical subset of the JSF Air Vehicle (AV) C++ Coding
Standard. JSF AV was written for DO-178B Level A flight-critical embedded
software on pre-C++11 compilers; Aetherion is a C++23 simulation and GNC
research library built on the STL, Eigen, and CppAD. Rules that genuinely
improve correctness, readability, or maintainability are adopted regardless
of that difference in context. Rules that would require dismantling the
existing, working architecture for no safety benefit in this context are
explicitly waived below, with rationale, rather than silently ignored.

Rule numbers refer to the JSF AV C++ Coding Standards document
(stroustrup.com/JSF-AV-rules.pdf).

## Rule-Breaking Process

JSF AV Rules 4-7 govern how deviations from the *other* 217 rules get approved
and recorded. Rules 4 and 5 assume a multi-role organization (engineering lead,
product manager) that doesn't exist for a single-maintainer research library;
the practical equivalent adopted here is: a deviation is "approved" by having a
reasoned, written entry in this document, authored by the maintainer. There is
no separate sign-off step beyond that.

- **Rule 6** ("each deviation from a `shall` rule shall be documented in the
  source file") is applied selectively rather than literally. Pervasive,
  architecture-wide waivers (exceptions, STL, dynamic allocation, `auto`,
  naming convention) are documented centrally in this file only — annotating
  every one of, say, the 130+ `auto` usages would be noise, not signal. The
  handful of *contained, non-obvious* deviations get an inline comment plus a
  pointer back here, so a future contributor doesn't mistake a deliberate
  choice for an oversight and "fix" it: `ArgumentParser.h`'s `std::exit`
  default, `Log.h`'s logging macros, `SE3.h`'s `.inl` include, and the
  CMake-injected `#define` path-constant pattern in the ~14 example/test files
  listed under Rule 30 below. Self-evidently standard idioms (`__has_include`,
  `NDEBUG`, `NOMINMAX`, `CATCH_CONFIG_MAIN`) are left uncommented since they
  need no extra explanation to any C++ engineer.
- **Rule 7** (no approval needed for deviations already covered by an explicit
  exception) is exactly how this document is meant to be used going forward:
  once a pattern is listed below as waived, continuing it doesn't require
  re-litigating the decision per call site.

## Functions / Complexity

| Rule | Summary | Status |
|---|---|---|
| 1 | Functions shall not exceed 200 LSLOC | **Enforced** — `.clang-tidy`: `readability-function-size.LineThreshold=200` (hard CI gate via `clang_tidy.yml`). `metrixpp.yml` reports LOC informationally. |
| 2 | No self-modifying code | **Compliant** — grepped for `VirtualProtect`/`mprotect`/`PROT_EXEC`/`dlopen`/JIT-codegen patterns project-wide; zero hits. |
| 3 | Cyclomatic complexity ≤ 20 | **Enforced** — `.clang-tidy`: `readability-function-size.CyclomaticComplexityThreshold=20` (hard CI gate). `metrixpp.yml` cross-checks at the same threshold, informationally. |
| 105/129 | No recursion | **Compliant by audit** — codebase uses iteration/templates throughout; not mechanically enforced (`misc-no-recursion` is disabled in `.clang-tidy` to avoid flagging template metaprogramming false positives). |
| 110 | ≤ 7 parameters per function | **Compliant by audit** — no functions found exceeding 7 params; no direct clang-tidy check exists for this, not tooled. |
| 113/123 | Single point of exit / no recursion (reinforced) | **Waived** — early-return and loop-`break`/`continue` exits are idiomatic and used throughout numerical code (Newton solvers, trim solver, simulator loops); forcing single-exit would require flag-variable contortions that reduce readability for no safety benefit here. |
| 156 | No `goto` | **Compliant** — zero occurrences. |
| 157 | No `continue` | **Waived** — one occurrence (`Aetherion/ODE/RKMK/Core/Newton.h:125`), an idiomatic early-continue in a damping loop. |
| 158 | `break` only in `switch` | **Waived** — six occurrences, all idiomatic loop early-exits (`Newton.h:144,148`, `Atmosphere.h:132`, `TrimSolver.h:251`, `ISimulator.h:121`). The codebase has zero `switch` statements. |

## Comments

| Rule | Summary | Status |
|---|---|---|
| 131 | File header comment (purpose/author/date/version) | **Compliant by convention** — every file carries a `Project/Copyright/SPDX-License-Identifier` banner. |
| 132 | Function header comment (purpose/inputs/outputs) | **Compliant by convention** — Doxygen `///`/`@brief`/`@param` comments are used project-wide; not exhaustively audited per-function, not mechanically enforced. |
| 133 | Comments in English | **Compliant** — verified by spot-check. |

## Style

| Rule | Summary | Status |
|---|---|---|
| 40 | Implementation files shall include the headers for everything they use | **Compliant via existing mechanism** — same as Rules 36/37: `iwyu.yml` (include-what-you-use) flags missing includes, not just unnecessary ones, informationally. |
| 41 | Lines ≤ 120 characters | **Enforced** — `.clang-format`: `ColumnLimit: 120`, hard CI gate via `clang_format.yml`. |
| 42 | Each expression-statement on a separate line | **Not separately tooled** — clang-format doesn't reformat semicolon-separated statements already packed onto one source line; not observed in spot checks, but not exhaustively audited. |
| 43 | Avoid tabs | **Enforced** — `.clang-format`: `UseTab: Never`. |
| 44 | Indentation ≥2 spaces, consistent within a file | **Enforced** — `.clang-format`: `IndentWidth: 2`, applied uniformly by the formatter. |
| 45 | Words in an identifier separated by `_` | **Waived** — same naming-convention waiver as Rule 51/52 (camelCase methods, `k`-prefixed constants); see Identifiers/Naming below. |
| 53 / 53.1 / 54 | `.h` for headers (no `'`/`\`/`/*`/`//`/`"` in the name), `.cpp` for implementation | **Compliant** — one documented `.inl` exception (Rule 32); all filenames are plain alphanumeric. |
| 55/56 | File name reflects the logical entity it declares/defines | **Compliant by convention** — e.g. `F16AeroPolicy.h` declares `F16AeroPolicy`, `DAVEMLReader.cpp` defines `DAVEMLReader`'s members. |
| 57 | Class sections ordered public, protected, private | **Compliant, verified** — checked every `public:`/`protected:`/`private:` access-specifier across all of `Aetherion/`; every class follows public→protected→private, no exceptions found. |
| 58 | Multi-parameter functions: first arg on the same line, rest one per line | **Enforced** — `.clang-format`: `BinPackArguments: false`, `BinPackParameters: false`, `AllowAllParametersOfDeclarationOnNextLine: false` produce exactly this layout when wrapping is needed. |
| 59 | Braces always present on `if`/`else`/`while`/`do`/`for` bodies | **Not enforced, and not followed everywhere** — `readability-braces-around-statements` was already disabled in `.clang-tidy` before this audit, and brace-less single-statement bodies are in active use (e.g. `TrimSolver.h:216-217`, `if (cond) return x;`). Documented honestly rather than claiming compliance; not changed in this pass. |
| 60/61 | Braces on their own line, same column, nothing else on the brace line | **Enforced** — `.clang-format`: `BreakBeforeBraces: Custom` + the `BraceWrapping` block (`AfterClass`/`AfterFunction`/`AfterControlStatement`/etc. all `true`) is Allman-style brace placement, matching this exactly. |
| 62 | `*`/`&` directly connected to the type-specifier | **Enforced** — `.clang-format`: `PointerBindsToType: true`. |
| 63 | No spaces around `.`/`->`, or between unary operators and operands | **Enforced** — default behavior of the Google-based clang-format style; it never inserts such spaces. |

## Namespaces

| Rule | Summary | Status |
|---|---|---|
| 98 | Every non-local name except `main()` in a namespace | **Compliant by consistent practice** — every header read during this and prior audits wraps its declarations in `namespace Aetherion::...`; the only exceptions are `main()` itself (`EntryPoint.h`, explicitly exempted by the rule) and the `Log.h` macros (preprocessor text substitution, not language-level names, so the namespace rule doesn't apply to them — see Rule 29). |
| 99 | Namespaces nested no more than 2 levels deep | **Waived** — the project's namespace hierarchy mirrors its directory structure for navigability in a large numerical library (e.g. `Aetherion::ODE::RKMK::Core`, `Aetherion::FlightDynamics::Policies::F16` are 3-4 levels deep), the same pattern used by Eigen, Boost, and other large C++ libraries. Flattening it would be a sweeping, purely cosmetic rename touching virtually every file, for no correctness benefit. |
| 100 | Explicit qualification/using-declaration for few names; using-directive for many | **Compliant by convention** — namespace aliases (`namespace FD = Aetherion::FlightDynamics;`) and selective `using X::Y;` imports are used for a handful of names (e.g. `F16AeroPolicy.h`'s `using Environment::detail::SquareRoot;`), while broader `using namespace X;` appears only where many names from one module are needed (test files exercising a module's full public API). |

## Classes

| Rule | Summary | Status |
|---|---|---|
| 65/66 | Use `struct` when no invariant is enforced; `class` when one is | **Enforced via fix** — `Config` and `VectorField` had no enforced invariant (no `private:` section) and are now declared `struct`. |
| 67 | Public/protected data only in `struct`, not `class` | **Fixed for known violations** — `F16AeroPolicy` (control-surface state → `setControls()`/`xcgFromAcM()`), `DAVEMLAeroModel` (reference geometry + internal graph state → private + accessors), `DAVEMLReader` (`m_vars` → private). **Not yet an automated gate**: `misc-non-private-member-variables-in-classes` remains disabled in `.clang-tidy` because the ~50 files containing `class` declarations (mostly `Aetherion/Examples/*Application.h`/`*Simulator.h`) have not been exhaustively audited; enabling it blind risks an unverifiable CI break. Tracked as a follow-up. |
| 69 | Const member functions where state isn't modified | **Enforced** — `readability-make-member-function-const` (already active). |
| 70 | `friend` only when necessary | **Compliant by audit** — one justified use (`SE3::operator*`). |
| 76/79 | Rule of 5 / resource release in destructor | **Compliant by audit** — no raw owning pointers found; non-owning pointers (`TrimSolver::m_aero`/`m_prop`) are already private. |
| 78 | Virtual destructor on classes with virtual functions | **Enforced** — `cppcoreguidelines-virtual-class-destructor` (already active); verified compliant (`Application`, `ISimulator`). |
| 88 | Multiple inheritance restrictions | **Compliant by audit** — no multiple inheritance in the codebase. |

## Identifiers / Naming

| Rule | Summary | Status |
|---|---|---|
| 46 | Identifiers shall not rely on significance beyond 64 characters | **N/A** — no length limit on identifier significance in any compiler this project targets (MSVC/Clang/GCC), unlike the 1990s embedded compilers JSF AV targeted. |
| 47 | Identifiers shall not begin with `_` | **Compliant** — verified by audit; no leading-underscore identifiers in project code (reserved-name territory per the C++ standard anyway). |
| 48 | Identifiers shall not differ only by case/`_`/look-alike characters | **Compliant by convention** — not mechanically enforced (no direct clang-tidy check); the project's consistent per-category casing (below) makes accidental near-duplicates unlikely. |
| 49 | Acronyms in an identifier shall be uppercase | **Compliant by convention** — `DAVEML*`, `WGS84`, `SE3`, `FMU`, `RKMK`, `NED`/`ECI` are consistently uppercase project-wide. |
| 50 | Types (`class`/`struct`/`namespace`/`enum`/`typedef`) start uppercase | **Compliant** — this is exactly the project's existing `PascalCase` type-naming convention. |
| 51 | Function and variable names entirely lowercase | **Waived** — the project uses an internally consistent, different convention instead: `camelCase` for methods/members (`setControls`, `getConfig`, `m_xcgFromAcM`), and `snake_case` with physical-unit suffixes for plain-data fields that mirror flight-dynamics literature (`el_deg`, `vt_fps`, `alpha_deg`). Rewriting this project-wide convention to bare lowercase would touch effectively every identifier in ~250 files for no readability or correctness benefit. |
| 52 | Constants and enumerators lowercase | **Waived** — the project uses `k`-prefixed `PascalCase` constants (`kFt_m`, `kMaxIter`, `kTol`), a long-established, unambiguous C++ convention already universal in this codebase; same rationale as Rule 51. |

## Templates

| Rule | Summary | Status |
|---|---|---|
| 101 | Templates reviewed both in isolation and for all actual instantiations | **Compliant by process** — a review/process rule rather than something a tool checks; the concept-constrained design (Rule 103) makes most actual instantiations type-check or fail to compile at the instantiation site, which substantially narrows what "review for all instantiations" needs to additionally cover. |
| 102 | Tests shall cover all actual template instantiations | **Compliant by extensive existing practice** — the dual-scalar pattern (`S = double` and `S = CppAD::AD<double>`, used everywhere CppAD-differentiable code is needed) is systematically exercised for both instantiations across dozens of test files (`test_gravity.cpp`, `test_atmosphere.cpp`, `test_MachNumber.cpp`, `test_AerodynamicMoments.cpp`, `test_AerodynamicAngles.cpp`, `test_AerodynamicForces.cpp`, `test_GravityPolicies.cpp`, `test_WindModels.cpp`, `test_TrimSolver.cpp`, and more). Not exhaustively audited instantiation-by-instantiation against every template in the codebase. |
| 103 | Constraint checks should be applied to template arguments | **Compliant by extensive existing practice** — the codebase has a deliberate, layered C++20 concepts infrastructure: `Aetherion/ODE/RKMK/Core/Concepts.h` (`ValueSemantics`, `ScalarLike`, `ADCompatibleScalar`, `EigenVectorLike`, `LieGroup`, `XiField`/`EuclidField`/`ProductDynamics`), `Aetherion/ODE/RKMK/Concepts.h` (`KinematicsFieldOnSE3`, `VectorFieldOnProductSE3`, `RKMKIntegratorOnProductSE3`, `IntegratorFor`), and `Aetherion/FlightDynamics/Policies/PolicyConcepts.h` (`GravityPolicy`/`AeroPolicy`/`PropulsionPolicy`/`MassPolicy`) — e.g. `VectorField`'s template parameters are concept-constrained (`FD::GravityPolicy Gravity, FD::AeroPolicy Aero = ...`). Many lower-level `template<class S>` member-function templates are left unconstrained where the constraint is already implied by the enclosing concept-constrained class or policy type, rather than redundantly repeated per method. |
| 104 | Template specialization declared before its use | **Compliant, verified** — checked the two explicit-specialization sites: `SnapshotTraits<SnapshotFormat::F>` (`Aetherion/Simulation/SnapshotTraits.h`) declares the primary template as intentionally-undefined before all 6 specializations follow in the same file; `is_wind_model<W>` (`Aetherion/Environment/WindModels.h`) declares its primary template before its own in-file specializations, and the out-of-file specialization in `GeodesicCallbackWind.h:100` correctly `#include`s `WindModels.h` first — its only use site (`AeroPolicies.h`'s `static_assert(is_wind_model_v<Wind>, ...)`) is itself a template not instantiated until a concrete `Wind` type is supplied by calling code, by which point the relevant specialization header is already included. |

## Language / Character Set

| Rule | Summary | Status |
|---|---|---|
| 8 | Conform to ISO/IEC 14882:2002 (C++98) | **Waived** — the project deliberately targets C++23 (`CMAKE_CXX_STANDARD 23`, required) for concepts, ranges, `std::numbers`, structured bindings, etc. Conforming to C++98 would mean abandoning the language version the entire design is built on. |
| 9 | Basic source character set only | **Waived** — Doxygen comments use Unicode math/Greek/box-drawing characters extensively (`α`, `β`, `×`, `→`, `½ρV²`, `──` section dividers — confirmed present in 59 files) for equation readability in a flight-dynamics library full of literal equations. Verified these never appear inside string literals or identifiers (zero matches), so there is no runtime character-handling impact — purely a documentation-readability choice, not realigned. |
| 10 | Character values restricted to a defined ISO 10646-1 subset | **N/A** — no runtime processing of exotic character data; the only non-ASCII content is static comment text (see Rule 9). |
| 11 | No trigraphs | **Compliant, moot** — trigraphs were removed from the language entirely in C++17; not expressible in a C++23 codebase. |
| 12 | No digraphs | **Compliant** — zero usage found (`<%`, `%>`, `<:`, `:>`, `%:`). |
| 13 | No multi-byte characters / wide string literals | **Partially waived** — the UTF-8 multi-byte comment content from Rule 9 is the same deliberate exception. Wide string literals (`L"..."`): zero usage found, fully compliant on that part. |
| 14 | Literal suffixes uppercase (`64L` not `64l`) | **Compliant** — zero lowercase-suffixed numeric literals found project-wide. |
| 15 | Provision for run-time checking (defensive programming) | **Compliant by convention** — extensive use of `static_assert` against C++20 concepts at compile time (53 occurrences across 18 files, e.g. `static_assert(AeroPolicy<F16AeroPolicy>)`), plus exception-based runtime validation throughout the serialization/parsing code (`DAVEMLReader`, `DAVEMLAeroModel`, `LoadConfig`). Same exception-based mechanism as the rest of the error-handling design (see Rule 208/215 waiver). |

## Libraries

| Rule | Summary | Status |
|---|---|---|
| 16 | Only DO-178B Level A / SEAL 1 certifiable libraries with safety-critical code | **Waived** — Aetherion is a research/simulation library, not a certified safety-critical product; it depends on Eigen, CppAD, yaml-cpp, nlohmann_json, spdlog, none of which are DO-178B certified. Adopting this rule would mean dropping the entire dependency stack. |
| 17 | No `errno` | **Compliant** — zero usage found. |
| 18 | No `offsetof` | **Compliant** — zero usage found. |
| 19 | No `<locale.h>`/`setlocale` | **Compliant** — zero usage found. |
| 20 | No `setjmp`/`longjmp` | **Compliant** — zero usage found. |
| 21 | No `<signal.h>` | **Compliant** — zero usage found. |
| 22 | No `<stdio.h>` I/O library | **Fixed** — the only usage was `std::remove()` (from `<cstdio>`) for temp-file cleanup in 3 test files (`test_Application.cpp`, `test_LoadConfig.cpp`, `test_DAVEMLReader.cpp`); replaced with `std::filesystem::remove()`, already an established dependency elsewhere in the codebase (`ConfigNlohmannAdapter.h`, `Adapter.h`). |
| 23 | No `atof`/`atoi`/`atol` | **Compliant** — zero usage found. |
| 24 | No `abort`/`exit`/`getenv`/`system` | **One justified, isolated exception** — `Aetherion::Simulation::ArgumentParser` defaults its `--help`/`-h` handler to `std::exit`, but this is already injected via an `ExitFn = std::function<void(int)>` constructor parameter specifically so it can be overridden (the header comment already notes "tests may supply a throwing stub") — the design already anticipated and isolated this concern before this audit. Not changed further. |
| 25 | No `<time.h>` functions | **Compliant** — zero usage found. |

## Pre-Processing Directives

| Rule | Summary | Status |
|---|---|---|
| 26-28, 31 | Only `#ifndef`/`#define`/`#endif`/`#include`, used only for include guards | **Mostly compliant, with documented exceptions below** — header guards use `#pragma once` (see Rule 35), so the codebase doesn't even use `#ifndef`-style guards; the few remaining `#if`/`#ifdef`/`#define` uses are all for things the preprocessor is the *only* mechanism for (next row), not improvised control flow. |
| 29 | No inline macros; use inline functions | **Audited, one known exception left as-is** — `Aetherion/Simulation/Log.h` defines 10 variadic logging macros (`AE_CORE_TRACE`/`AE_INFO`/etc.) forwarding to `spdlog::logger`. These are called unqualified from many different namespaces (`Aetherion::Simulation`, every `Aetherion::Examples::*`), relying on the macro's hardcoded fully-qualified expansion — a literal function wouldn't be found unqualified from unrelated namespaces without `using`-declarations everywhere. A global-scope forwarding-template fix is possible but its interaction with fmt/spdlog's compile-time format-string validation is version-sensitive; not changed blind without a compiler available to verify it. |
| 30 | No `#define` for constants; use `const` instead | **One justified exception** — `#ifndef F16_AERO_FILE / #define F16_AERO_FILE "" / #endif` (and similarly named) in ~13 example/test files are CMake `-D`-injected path constants (set via `target_compile_definitions` in the corresponding `CMakeLists.txt`) with a fallback default. Command-line/build-time string injection into C++ source has no non-preprocessor mechanism. |
| — | (supporting evidence) | Other non-guard preprocessor use, all inherent to their purpose: `#if __has_include(<cppad/cppad.hpp>)` (optional-dependency detection, `State.h`/`Math.h`/`SE3.cpp`), `#ifdef NDEBUG` (debug/release branch, `Log.cpp`), `#if defined(__cpp_lib_to_chars)` (standard-library feature-test, `tests/Utility/parse_utils.h`), `#define CATCH_CONFIG_MAIN` (mandated by the Catch2 framework contract), `#define NOMINMAX` / `#undef max`/`min` (standard Windows.h macro-pollution workaround, `test_Skew.cpp`). None of these have a non-preprocessor equivalent. |
| 32 | `#include` shall only include `.h` files | **One justified exception** — `Aetherion/ODE/RKMK/Lie/SE3.h` `#include`s `SE3.inl`, the standard declaration/`.inl`-implementation split used for heavily-templated headers (the same role a `.cpp` plays for non-template code, here separated because the implementation is itself templated and must stay header-visible). Otherwise every project file is `.h`; third-party headers with their own conventions (`<Eigen/Dense>`) aren't ours to change. |

## Header Files

| Rule | Summary | Status |
|---|---|---|
| 33 | `#include` shall use `<filename.h>` notation | **Mixed, not realigned** — `Aetherion/` library headers are consistently included via angle brackets with full project-relative paths (`#include <Aetherion/.../X.h>`) project-wide. Same-directory sibling includes in `src/`/`tests/` (and a handful of headers) use quotes (`#include "X.h"`), a common and reasonable angle="library"/quote="local" convention. Not mechanically realigned to the letter of the rule — touches ~68 files and risks include-path resolution mistakes for a purely stylistic gain. |
| 34 | Headers should contain only logically related declarations | **Compliant by design** — one primary type/policy per header is the established pattern throughout `Aetherion/`. |
| 35 | Every header shall contain an include guard | **Compliant via modern equivalent** — `#pragma once` is used universally instead of `#ifndef`/`#define` guards; supported by all three target compilers (MSVC/Clang/GCC) and serves the same purpose. |
| 36/37 | Minimize compilation dependencies / only include what's needed | **Partially addressed** — `.github/workflows/iwyu.yml` (include-what-you-use) reports unnecessary/missing includes informationally; not a hard CI gate. |
| 38 | Forward-declare classes only accessed via pointer/reference | **Not separately audited** — many pointer/reference members (e.g. `TrimSolver::m_aero`) still require the full definition in-header because they call template member functions, which can't work with a forward declaration alone. Left to IWYU's informational report rather than a manual pass. |
| 39 | Headers shall not contain non-const variable definitions or function definitions | **Waived** — Aetherion is a header-only template library (`Aetherion/` has 0 `.cpp` files); inline function definitions in headers (including the `setControls()`/accessor methods added by this pass) are required by the architecture, not an oversight. |

## Pointers, Casts, Memory, Exceptions — explicitly waived

These conflict with the existing, working architecture (STL/Eigen/CppAD,
exception-based error handling, template-heavy generic code) and are not
adopted. The codebase already happens to use few of the banned constructs
(zero `dynamic_cast`/`reinterpret_cast`/`const_cast`, minimal C-style casts),
but the blanket bans below are not enforced:

- **206** No dynamic heap allocation, **221** no STL in safety-critical code — incompatible with an STL/Eigen-based design.
- **208/215** No C++ exceptions — the codebase's error handling (`DAVEMLReader`/`DAVEMLAeroModel` parsing, etc.) uses `std::runtime_error`/`std::out_of_range` by design.
- **144** No `auto` — used extensively (130+ occurrences) and is idiomatic modern C++; banning it would reduce readability.
- **186/187/188** No `dynamic_cast`/`reinterpret_cast`/`const_cast` — moot in practice (zero usage found) but not enforced as a hard ban, to leave room for future legitimate use.

## Tooling reference

- `.clang-tidy` — primary enforcement mechanism; `WarningsAsErrors` mirrors `Checks`, so every enabled check is a hard CI gate via `.github/workflows/clang_tidy.yml`.
- `.github/workflows/metrixpp.yml` — informational/secondary cyclomatic-complexity report, cross-checked against the same threshold as the clang-tidy gate.
