# JOSS submission

Short software paper for the *Journal of Open Source Software*, condensed from
`softwarex/` (now sidelined). JOSS goes first; the AST formulation and
verification paper then cites the software by its JOSS DOI:

> The framework is described in [JOSS]; here we present the formulation and
> verification.

So this paper says *what the software is and why it is built that way*, and
leaves the derivations, convergence study, constraint-drift study and the
verification table to AST. Keep it that way when editing.

## Files

| File | |
|---|---|
| `paper.md` | Manuscript (~1 150 words; JOSS limit 750–1 750) |
| `paper.bib` | References |
| `figures/architecture.tex` | TikZ source of the only figure |
| `figures/architecture.png` | Rendered figure (tracked; `*.pdf` is gitignored) |

Regenerate the figure:

```sh
cd figures
pdflatex architecture && pdftoppm -png -r 300 -singlefile architecture.pdf architecture
```

The architecture diagram is JOSS-only. Do not reuse it in the AST paper; draw
a new figure there.

## Sections

JOSS (2025 guidelines) requires: Summary, Statement of need, State of the
field, Software design, Research impact statement, AI usage disclosure,
Acknowledgements, References.

## Building and submitting

JOSS reviews `paper.md` from the software repository, so copy `paper.md`,
`paper.bib` and `figures/architecture.png` into the Aetherion repository
(e.g. `paper/`) before submitting. Preview there with the Open Journals
`draft-pdf` GitHub Action (`openjournals/openjournals-draft-action`), or
locally with Docker:

```sh
docker run --rm -v "$PWD:/data" -u $(id -u):$(id -g) \
    --env JOURNAL=joss openjournals/inara
```

## Differences from the SoftwareX text

- TM-2015-218675 is cited correctly as Murri, Jackson and Shelton, *Check-Cases
  for Verification of 6-Degree-of-Freedom Flight Vehicle Simulations*. The
  `NASA_TM_2015_218675` entry in the other papers' `references.bib` names
  Madden's *Equations of Motion* report instead and needs the same fix.
- Eigen is cited.
