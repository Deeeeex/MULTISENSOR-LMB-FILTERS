# GCE V2X diagnosis

Start with `DIAGNOSIS_CN.md`. The preceding temporal association study is committed and closed at `e661f2e88764dd90a2d6bfcab6a000ef3bcaba05`.

This directory contains a post-outcome diagnosis, not a new fusion method. All five earlier V2X validation segments and all fourteen additional test segments are retained, with both radio conditions. Existing full recursive GCE and No-age KLA trajectories supply 76 native outputs. No detector, calibration, tracker, radio draw or manuscript is changed.

`diagnose_gap.py` independently recomputes paired frame scores, an exact additive OSPA decomposition, fixed-current-input substitutions, detection support and continuous No-age-only intervals. `--preflight v2xt_0005` checks one completed pair before the full read-only analysis. Full execution requires the recorded base commit and refuses to overwrite `GAP_DIAGNOSIS.json` or `DIAGNOSIS_FREEZE.json`.

`diagnose_tracks_v2.py` is the canonical detailed trace analyzer. The original `diagnose_tracks.py` is preserved with its failed log and `TRACE_SELECTION.json`. One selected loss window had only extra false alarms and no missed target. `CASE_SELECTION_REPAIR.json` records the bounded extension that keeps this window and traces its most frequent unmatched native label. The native files and phase-one scores are unchanged. `TRACE_SELECTION_V2.json` records the final seven case choices.

Scientific execution uses `tmp/external_baselines/v2v_inference_venv/bin/python`. `verify_diagnosis.py --full` checks all source/result hashes and native trace fields without importing study code. `verify_diagnosis.py` alone independently recomputes all portable CSV and JSON arithmetic without native files. The full verification receipt is `DIAGNOSIS_VERIFICATION.json`.

Plotting uses the existing Python runtime at `tmp/external_baselines/venv/bin/python`, which includes matplotlib. `plot_diagnosis.py` writes editable SVG, PDF, PNG and complete source series under `figures/`. `FIGURE_CONTRACT.md` states the evidence, time windows and export scope. `build_report.py` renders the Chinese report only after full verification.

`diagnose_fragmentation.py` counts all post-fusion components within 2 m of the already selected longest loss case, distinguishing pruning from surviving low-probability labels. `FRAGMENTATION_DIAGNOSIS.json` and `fragmentation_frames.csv` retain every frame; these are counts from existing trajectories, with no new method run.

The fixed-input substitutions leave GCE's prior recursion intact and cannot establish a recursively executed improvement. Positive/negative removal sensitivities are reported together with localization, missed-target and false-target terms. Target matches are offline annotation diagnostics; geometric sensor coverage does not establish physical visibility. Original/native result hashes are preserved.

`output/` contains a compact source/evidence archive and the fresh-extraction verification receipt. The package includes the diagnostic sources, derived data and figures, not raw point clouds, model weights or native trajectories. Its portable check recomputes the report arithmetic and regenerates the figure with the recorded Python environment; it does not rerun MATLAB.
