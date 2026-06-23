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
- `scripts/verify_n50_evidence.py`: independently recomputes network disagreement from per-trial report rows and runtime from the trial log.
- `scripts/render_figures.py`: dependency-light figure renderer.
- `../../../RUN/AA/launchAaTaesN50LocalVerifierRerun.sh`: starts the long N50 rerun that emits per-trial local tracking rows for final local-metric verification.
- `generated/`: reproducibility fragments generated from `RUN/AA/AA_BALANCED_CARDINALITY_VALIDATION_N50_SEED1_20260622_174819.md`.
- `figures/`: generated figure assets.

Build:

```bash
./build.sh
```

The build first regenerates the N50 evidence fragments, runs the verifier, renders figures, then compiles the TAES PDF. Do not edit `generated/n50_*.tex` by hand; update the validation report or extraction script and rerun the build.

The verifier currently recomputes network disagreement and runtime from raw per-trial artifacts. The validation runner now emits per-trial local E-OSPA/RMSE/CardErr rows for new reports, and the verifier will independently recompute those metrics when the source report contains that table. The archived N50 report used by the current draft predates that table, so its local tracking metrics remain trace-checked through the report summary and generated evidence JSON until the N50 validation is rerun.

Long N50 rerun for upgrading local metric verification:

```bash
../../../RUN/AA/launchAaTaesN50LocalVerifierRerun.sh
```

After the rerun completes, update the source report/log constants in `scripts/extract_n50_evidence.py` and `scripts/verify_n50_evidence.py`, then run `./build.sh`.

The current manuscript is not yet submission-ready: author information is provisional, figures still need final design polish, and broader scenario plus rerun N50 local-metric verification still need to be added before final submission.
