# TAES Manuscript Draft

This directory contains the TAES-first manuscript source for the AA label-barycenter work.

Current target:

```text
IEEE Transactions on Aerospace and Electronic Systems
Manuscript type: Regular Paper
Technical area: Target Tracking and Multi-Sensor Systems
```

Main files:

- `main.tex`: manuscript draft.
- `references.bib`: DOI-verified core references used by the draft.
- `IEEEtaes.cls`, `IEEEtaes.bst`: official TAES template files.
- `scripts/extract_n50_evidence.py`: parses the tracked N50 validation report and generates manuscript table/figure fragments.
- `scripts/render_figures.py`: dependency-light figure renderer.
- `generated/`: reproducibility fragments generated from `RUN/AA/AA_BALANCED_CARDINALITY_VALIDATION_N50_SEED1_20260622_174819.md`.
- `figures/`: generated figure assets.

Build:

```bash
./build.sh
```

The build first regenerates the N50 evidence fragments, then renders figures, then compiles the TAES PDF. Do not edit `generated/n50_*.tex` by hand; update the validation report or extraction script and rerun the build.

The current manuscript is not yet submission-ready: author information is provisional, figures still need final design polish, and broader scenario/independent-verifier evidence still needs to be added before final submission.
