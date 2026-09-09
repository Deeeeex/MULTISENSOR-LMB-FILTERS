# Recursion origin and one-time intervention

This isolated diagnostic continues the already closed range-detection and
joint-admission studies. It does not reopen their method screens or modify
the original fusion implementation, datasets or manuscript.

`PROTOCOL.md` and `TRACE_FREEZE.json` bind a descriptive trace over twelve
existing native trajectories. `TRACE_VERIFICATION.json` independently
checks 5760 target robot frames. The original `trace.py` and its initial
CSV files remain frozen. `TRACE_SERIALIZATION_REPAIR.json` explains the
bounded `trace_v2.py` change for missing inactive-source existence values;
the two CSV files are byte-identical before and after that repair.

`INTERVENTION_PROTOCOL.md` declares the subsequent causal experiment in
one previously exposed range/intermittent case. `INTERVENTION_FREEZE.json`
binds both configurations before any new native outcome: two exact
baseline controls and three full trajectories with an event at frame 3.
`RUNNER_PATCH.json` preserves the exact changes from the range replay.
`AUDITOR_PATCH.json` preserves the independent Gaussian reconstruction
and identifies its sole new event branch. All native processes run the
analytic `checkRecursionIntervention` fixture before their real trajectory.

The run order is fixed:

1. `run_stage.py recursion_preflight --workers 1`
2. `audit_stage.py recursion_preflight`
3. `run_stage.py recursion_interventions --workers 2`
4. `audit_stage.py recursion_interventions`
5. `describe_events.py`, `build_report.py`, then `verify_archive_v2.py`

Use `python3` for the native launcher. Scientific analysis uses the existing
`tmp/external_baselines/v2v_inference_venv/bin/python`, which contains NumPy
and SciPy. Native jobs use MATLAB R2024a. The drivers and result builders
refuse to overwrite registered runs or completed receipts.

Native logs are under `RUN/ICRA_RECURSION_ORIGIN/<stage>/<execution_id>.log`.
Each runtime ledger is written only after `process.wait()` returns, and
records exit status, completion marker and exact output count. Native
`.json.gz` files remain in the ignored `results/` directory; final checks
bind their hashes along with the immutable sources and existing references.

`RESULTS_CN.md` reports all six trajectories (five new, one audited No-age
reference), without choosing a candidate algorithm. The complete scores
are in `ALL_INTERVENTION_SCORES.csv`; the two `target_frames_*.csv` files
contain all 2880 target robot frames, candidate counts and association
concentration. `FINAL_VERIFICATION.json` checks the archived sources,
ports and report totals, and independently recounts the native target and
association diagnostics. Its success denotes completion of this bounded
causal diagnostic only.

`ARCHIVE_CHECKER_REFINEMENT.json` preserves an unused checker draft that
assumed at least 40 baseline fields. The current checker verifies the exact
original field set (39 fields excluding runtime) instead. Completed audits
and native configurations are unchanged. `EVENT_DETAILS.json` describes
the magnitude and components of every already declared intervention.
