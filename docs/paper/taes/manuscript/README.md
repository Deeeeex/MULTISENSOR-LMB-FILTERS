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
- `scripts/render_figures.py`: dependency-light figure renderer.
- `figures/`: generated figure assets.

Build:

```bash
./build.sh
```

The current manuscript is a first paper-source checkpoint. It is not yet submission-ready: author information is placeholder, figures are first-pass assets, and broader scenario/independent-verifier evidence still needs to be added before final submission.
