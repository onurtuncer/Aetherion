# SoftwareX reference papers

Recent SoftwareX articles, kept as formatting and style references for the
submission in the parent directory.

## Provenance

SoftwareX is fully open access (CC-BY), but ScienceDirect returns HTTP 403 to
non-browser clients, so the publisher PDFs could not be retrieved directly.
These are the **author preprints on arXiv** of articles that were subsequently
published in SoftwareX — matched by the `journal_ref` field, so each is
confirmed published in the volume shown. Content matches the published version;
the typesetting is the authors' own rather than Elsevier's production layout.

For the *authoritative* layout, the template used for `../main.tex` is the
SoftwareX article template Version 4 (November 2023), obtained as a pristine
copy from a published author's repository
(`pachadotdev/cpp11armadillo`, `softwarex-article/template.tex`).

## Papers

| File | Volume | Title |
|---|---|---|
| `arxiv_2603.11369.pdf` | 35 (2026), 102872 | abx_amr_simulator: A simulation environment for antibiotic prescribing policy optimization under antimicrobial resistance |
| `arxiv_2509.24340.pdf` | 35 (2026), 102827 | humancompatible.detect: a Python toolkit for detecting bias in AI models |
| `arxiv_2512.13268.pdf` | 34 (2026) | SPARS: A reinforcement learning-enabled simulator for power management |
| `arxiv_2512.20642.pdf` | 34 (2026), 102641 | Flow Gym: A framework for development, benchmarking and training |
| `arxiv_2512.09664.pdf` | 34 (2026), 102642 | SynthPix: A lightspeed PIV image generator |
| `arxiv_2506.20539.pdf` | 31 (2025), 102286 | Resolvent4py: a parallel Python package for analysis and model reduction |
| `arxiv_2410.19438.pdf` | 29 (2025), 102035 | eminus — Pythonic electronic structure theory |
| `arxiv_2405.14416.pdf` | 33 (2026), 102472 | VelCrys: Interactive web-based application to compute acoustic wave velocities |

The numerics/simulation-library papers (`Resolvent4py`, `SynthPix`, `eminus`,
`VelCrys`) are the closest analogues to the Aetherion submission in scope and
structure.

## Latest issue

At the time of writing (August 2026) the most recent SoftwareX volume is
**Volume 35 (September 2026)**, 120 articles. Full listing retrievable from the
CrossRef API:

```sh
curl "https://api.crossref.org/journals/2352-7110/works?rows=120&sort=published&order=desc"
```
