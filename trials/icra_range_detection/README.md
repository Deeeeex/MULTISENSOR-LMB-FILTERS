# Shared range-calibrated detection model

**Closed: the frozen screen failed three of its eight gates.** All 192 native
recursions and 73,092 robot-frames completed and passed their reconstruction
audits. Range GCE worsened V2V macro OSPA versus nominal GCE, and it lost to
the fitted-constant GCE control in both cohorts. See `RESULTS_CN.md` and
`SCREEN_SELECTION.json`; do not retune the curve or extend this candidate.

The complete recursive comparison and stopping rule are frozen in `PROTOCOL.md`.
This trial changes the shared observation model, so each proposed-model result
must be compared with No-age and Guarded Scalar using that same model.

The original replay was copied with bounded, recorded replacements in
`make_runner.py` and `RUNNER_PATCH.json`. The only numerical model change is
`range_quality/evaluateSensorQuality.m`. Local filtering, association, packet
formats and fusion implementations are reused from the original study.

`checkRangeDetection.m` verifies the actual local missed and single-detection
update against analytic Bernoulli formulas at 5, 20, 39 and 41 metres, for
nominal, fitted constant and range models, plus original geometric exclusions.

New diagnostic records contain:

- `qualityRecords`: frame, source, original birth label (two fields), predicted
  four-state mean, range, actual probability. Includes every pre-update object.
- `fusionSourceRecords`: frame, receiver, output label, local original label,
  remote original label. Absent sources have zero labels. These supplement
  No-age's legacy diagnostic rows without changing transmitted packets.
- `fusionOutputRecords`: frame, receiver, output label, existence, complete
  four-state mean and ten lower-triangular covariance entries in column order.

Commands from the repository root (no package installation required):

```sh
python3 trials/icra_range_detection/register_stage.py preflight
python3 trials/icra_range_detection/register_stage.py screen
python3 -u trials/icra_range_detection/run_stage.py range_detection_preflight --workers 2
tmp/external_baselines/v2v_inference_venv/bin/python -u trials/icra_range_detection/audit_stage_v2.py range_detection_preflight
tmp/external_baselines/v2v_inference_venv/bin/python -u trials/icra_range_detection/summarize.py --preflight-only
python3 -u trials/icra_range_detection/run_stage.py range_detection_screen --workers 2
tmp/external_baselines/v2v_inference_venv/bin/python -u trials/icra_range_detection/audit_stage_v2.py range_detection_screen
tmp/external_baselines/v2v_inference_venv/bin/python -u trials/icra_range_detection/summarize.py
```

Each command refuses to overwrite a completed stage or analysis. The runtime
ledger records process exit and completion markers; native outputs are retained
under the ignored `results/` directory. Source manifests bind all model code,
calibration, inputs, references and initially declared auditors before execution.

`METADATA_FIELD_REPAIR.json` records a verification-only correction: MATLAB
prefixes digit-leading diagnostic metadata keys (`5` becomes `x5`) when reading
the full calibration JSON. The v2 auditor checks all parameter values and
metadata exactly after that field-name conversion. The original auditor,
native algorithm, calibration values and manifests remain frozen.

The completed `finish_pipeline.py` receipt is `PIPELINE_EXECUTION.json`.
`verify_selection.py` independently recomputes all eight gates from native
OSPA arrays; `verify_final_archive.py` additionally binds the complete archive.

Post-outcome diagnostics are separate from method selection:

- `MECHANISM_NOTES_CN.md` and `MECHANISM_TRACE.json` describe the predeclared
  target's candidate distribution and output losses across the blackout.
- `SCALAR_CASE_SUBSTITUTION.json` tests current mean/normalizer replacements
  on Guarded Scalar's own visited inputs. `CASE_SUBSTITUTION_VERIFICATION.json`
  reproduces them through the saved GS density and transmitted ratios.
- `ASSIGNMENT_MASS_NOTES_CN.md` records conditional-association concentration
  and the zero positive mark on the fixed frame-53 nearest detection. This
  snapshot does not support blaming excessive positive admission from that
  detection; no new gate is implemented.
- `RECORDING_SENSITIVITY_CN.md` uses exact recording timestamps and collection
  dates for descriptive reweighting and leave-one-group-out checks. It does
  not alter the sequence-macro selection rule or create new holdout evidence.

Run the completed-case analysis and final additional checks with:

```sh
tmp/external_baselines/v2v_inference_venv/bin/python trials/icra_range_detection/diagnose_mechanism.py
tmp/external_baselines/v2v_inference_venv/bin/python trials/icra_range_detection/diagnose_scalar_case.py
tmp/external_baselines/v2v_inference_venv/bin/python trials/icra_range_detection/verify_case_substitution.py
tmp/external_baselines/v2v_inference_venv/bin/python trials/icra_range_detection/diagnose_assignment_mass.py
python3 trials/icra_range_detection/build_mechanism_note.py
python3 trials/icra_range_detection/recording_sensitivity.py
python3 trials/icra_range_detection/verify_final_archive.py
```

All result-writing commands refuse to replace existing completed artifacts.
