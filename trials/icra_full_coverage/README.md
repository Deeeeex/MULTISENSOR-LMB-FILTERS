# Complete V2V4Real release coverage

`PROTOCOL.md` fixes the counting and experiment before launch.
`DATA_INVENTORY.json` identifies every public archive scene and the duplicate
train/test sequence. `INPUT_MANIFEST.json` records the six added inputs.

The background chain runs an original-output parity preflight, both link
conditions and seven methods on all six missing scenes, independent saved
trajectory audits, and a complete 43-scene summary using the previous
verified results. The MATLAB entry point reuses the frozen replay body.

Progress: `BACKGROUND_STATUS.json` and `RUN/ICRA_FULL_COVERAGE/resume_master.log`.
Per-sequence native logs: `RUN/ICRA_REVIEWER_REVISION/coverage_*/`.
Completion: `COVERAGE_ANALYSIS.json`, `all_sequence_scores.csv`, `RESULTS_CN.md`.

The initial native preflight completed and all four trajectories matched.
Its auditor then needed the original helper paths for the provenance
report. `AUDIT_PATH_REPAIR.json` records the identical relative symlinks;
the original failure log and status are retained. The successful audit
checked 1,176 robot-frames. `resume_background.py` continues directly with
the six missing scenes, keeping those completed preflight outputs.

From the repository root, after preparation in a fresh output directory:

```sh
tmp/external_baselines/v2v_inference_venv/bin/python -u trials/icra_full_coverage/run_background.py
```

The scripts refuse to overwrite a previous execution. All complete native
traces remain under `results/`, excluded from Git. Existing manuscript
statistics continue to identify the completed 25-sequence comparison until
the additional complete-release report has passed.
