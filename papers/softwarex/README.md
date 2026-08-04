# SoftwareX submission

Journal companion to the two EUCASS conference papers:

- `papers/eucass-library/` — the broad library paper
- `papers/eucass/` — the SE(3) RKMK integrator paper

This article is the *software publication*: what the library does, how it is
put together, and what has been verified — rather than a derivation of the
numerics.

## Build

```sh
pdflatex main && bibtex main && pdflatex main && pdflatex main
```

Requires `elsarticle` (MiKTeX/TeX Live). Builds clean: no overfull boxes, no
undefined references.

## Format

Prepared with the **SoftwareX article template Version 4 (November 2023)**,
based on Elsevier's `elsarticle` class in `preprint,12pt,a4paper` mode.

Structure mandated by the SoftwareX Guide for Authors:

1. Metadata — the mandatory C1–C9 code metadata table
2. Motivation and significance
3. Software description (Software architecture / Software functionalities)
4. Illustrative examples
5. Impact
6. Conclusions
7. Declaration of competing interest, Acknowledgements, References

Body text is ~2 930 words excluding tables, figures and listings, which
corresponds to roughly the 6 published pages SoftwareX allows. The 15 pages of
the preprint PDF are an artefact of the template's single-column 12 pt layout.

Two deviations from the stock template, both marked with comments in
`main.tex`:

- `\parskip` is set to 6 pt. The template zeroes `\parindent` without a
  matching `\parskip`, so consecutive paragraphs run together visually.
- `microtype` is loaded with `expansion=false`. The template's title size
  resolves to a bitmap (pk) font and pdfTeX font expansion requires scalable
  fonts.

## Figures

`figures/convergence_order.pdf` and `figures/constraint_drift.pdf` are copied
from `papers/eucass/figures/`, together with the CSV data they are generated
from. Both are emitted by regression tests in the suite
(`tests/RigidBody/test_ConvergenceOrder.cpp` and
`tests/RigidBody/test_ConstraintDrift.cpp`) and plotted by
`papers/eucass/scripts/`.

## `reference-papers/`

Recent SoftwareX articles downloaded as formatting references. See
`reference-papers/INDEX.md`.
