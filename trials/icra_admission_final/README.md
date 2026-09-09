# Admission revision assessment

Read `RESULTS_CN.md` for the complete result. Original GCE remains the
working baseline: none of the nine new recursive admission candidates
produced a stable evaluation improvement. `EVALUATION_ANALYSIS.json`
contains all 854 final comparison rows; `all_evaluation_scores.csv` is the
flat version. The additional current-scan support screen did not qualify
for recursive evaluation.

The study preserves 458 new native trajectories, 187,444 independently
audited robot-frames, all candidate choices, source/input hashes and failed
registration logs. The protected original implementation is unchanged.
The original repository baseline for these additions is commit
`2c81bb5d` on `codex/icra`. Full native replay also requires its MATLAB
dependencies and the public datasets described in the included protocols.
Raw public point clouds and the large native trajectory archives are not
redistributed in the compact source package.

Recompute the compact summary with Python and NumPy:

```sh
python trials/icra_admission_final/verify_summary.py
```

This command also runs from a fresh extraction of
`output/icra_admission_assessment.zip`. It recomputes the per-segment,
per-recording and paired bootstrap summaries and the development ranking.
The original checkout additionally supports full file verification:

```sh
tmp/external_baselines/v2v_inference_venv/bin/python trials/icra_admission_final/accept_results.py --verify-only
```

`REPLAY_ACCEPTANCE.json` and `RESULT_FILES_MANIFEST.json` describe the
native evidence; `SUMMARY_AUDIT.json` describes the portable arithmetic
checks. `CSV_LINE_ENDINGS.json` records lossless line-ending normalization
of four unfrozen derived CSVs. `../icra_compatible_admission/DATA_EXPOSURE.json`
records the two train_0014 values displayed by a log tail before the
selection report; the candidate set and selection rule were unchanged.
The subsequent current-scan screen treats the already viewed V2X cohort
as development data.
