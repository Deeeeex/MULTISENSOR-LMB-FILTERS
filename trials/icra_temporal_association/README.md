# Cross-vehicle association study

This study starts from commit `dc6a4c14` and keeps the original detector,
local filter, Gaussian evidence fusion, datasets and radio draws fixed.
New messages expose current observation summaries; the receiver uses them
to inspect conflicting shared labels and, in S, retain a separate branch.
Q, N and QN then strengthen the evidence required by S. The fixed selected
version is QN, with confidence 0.9 and nominal discrepancy quantile 0.999.

The additional test is complete. QN has no reliable-link OSPA change and
an intermittent-link increase of 0.0000225147 m; no test segment improves.
No-age KLA has lower mean OSPA in both conditions. QN remains an experimental
version and is not promoted to the manuscript. Start with `DECISION_CN.md`.

## Valid result entry points

- `FINAL_RESULTS_CN.md`: canonical complete Chinese report, generated after
  the additional test audit and analysis finish. `RESULTS_CN.md` is
  retained as an earlier progress snapshot.
- `RESTORED_DEVELOPMENT_SELECTION.json`: completed, independently audited
  comparison of original GCE and D, T, R, S on all nine development segments.
- `CANDIDATES_V3.md`: frozen Q, N and QN comparison and its stronger exit gate.
- `RESTORED_ASSESSMENT_V2.json`: the complete S and matched Guarded Scalar
  assessment, produced only after every required stage exits and passes.
- `SCREENED_DEVELOPMENT_SELECTION.json`: complete V3 decision and all eight
  valid development settings. QN reduces OSPA by 1.4467% and 1.4322% on
  reliable and intermittent development links and passes its fixed gate.
- `SCREENED_ASSESSMENT_V2.json`: the complete exposed assessment, with 1,066
  rows including all 43 V2V segments and five previous V2X validation
  segments. QN's full V2V OSPA reductions are 0.9159% and 0.6841%; the
  previous V2X validation result is worse than original GCE in both links.
- `ADDITIONAL_TEST_ANALYSIS.json`: final additional test analysis, generated
  after all 140 native outputs passed. It compares No-age KLA, original
  GCE, Guarded Scalar, GCE + QN and Guarded Scalar + QN under both links.
- `REPLAY_ACCEPTANCE.json` and `RESULT_FILES_MANIFEST.json`: final native
  completion, source/input integrity and separate valid/invalid result
  inventories. Their creation requires every valid registered stage.
- `PROSPECTIVE_TEST_COHORT.json`: metadata-only freeze of all 14 paired
  mobile-vehicle segments in the additional official V2X-Real test archive.
  No result file above is evidence that this additional cohort has run.

S passes its original development gate with OSPA reductions of 0.7505%
and 0.1906% under reliable and intermittent delivery. D, T and R fail
that gate. The stronger V3 gate requires
at least 1% reduction in each condition and a nonincreasing scored wrong-pair
rate. Additional test results and matched fusion controls are required
before any method is promoted to the manuscript.

The additional test uses every prospectively registered segment: 14 segments,
2,172 frame pairs, five collection dates. Related segments are grouped by
date. QN is fixed before raw acquisition; test results do not select or tune
it. The primary comparison includes all segments. The predeclared secondary
breakdown uses only the known platform separation to distinguish overlapping
and nonoverlapping 40 m sensor disks.

## Invalid runs and implementation repairs

`PATH_FAILURE.json` and `PATH_RESTORATION.md` document a unit-check path
side effect that changed the observation-domain adapter. The initial
`association_v1_preflight`, `association_v1_development_rest` and
`association_v2_preflight` runs are retained for diagnosis. Their old
`DEVELOPMENT_SELECTION.json`, `V1_RESULTS_CN.md` and
`V1_FAILURE_ATTRIBUTION.json` do **not** support method comparisons.
Only the later restored stages support the D/T/R/S development decision.

The separate instrumentation replay has exact original-trajectory parity;
its fixed-original-input diagnostics remain valid. The restored runner
reinstalls and asserts the correct adapter after all unit checks. Its
auditor independently checks the rectangle, vehicle exclusion zones,
sensor range, executed detection probability and observation opportunity.

`ASSESSMENT_LOADING_REPAIR.json` records a heterogeneous-JSON unit-list
loading failure before any full-assessment output. `COLUMN_FAILURE.json`
and `COLUMN_SHAPE_PATCH.json` record a one-to-many array-shape repair,
also discovered before that unit produced an output. `runColumnAssociation`
uses explicit column division; native boundary checks establish exact
parity for the previously valid shapes. Failed runs and receipts are kept.

## Reproduction and interpretation

Use the registered JSON under `stages/`, the corresponding `run_*_stage.py`
entry point, and its matching independent auditor. Runners refuse to
overwrite a prior stage. Native stdout and errors live under
`RUN/ICRA_TEMPORAL_ASSOCIATION/<stage>/`; runtime ledgers record process
exit, completion marker and output count. Audits verify frozen source and
input hashes, the actual packet codec, causal receiver histories, global
assignment objectives, branch identifiers and both directions of source
abstention. Selection scripts consume completed audits only.

`PERSISTENT_METHOD_SPEC.md` describes R/S equations and state transitions.
`PROTOCOL.md`, `CANDIDATES_V1.md`, `CANDIDATES_V2.md` and
`CANDIDATES_V3.md` preserve the prospective decisions for each round.
The 2 m and 12 m ground-truth identity assignments are offline diagnostics;
they never enter the tracker. Their adjacent label changes are reported as
diagnostics, without claiming equivalence to a standardized MOT IDSW metric.

For the additional test, `audit_screen_assessment_unit.py` schedules the
unchanged v2 audit body over completed units while the slow final unit runs.
The exact wrapper replacements and old-benchmark equality check are in
`UNIT_AUDIT_SCHEDULING_PATCH.json` and `UNIT_AUDIT_PARITY.json`.
`merge_unit_assessment_audits.py` requires all native units and all result
hashes before producing the complete audit. The initial chaining attempt
`complete_additional_test.py` is preserved along with its analysis error;
the canonical corrected analysis is `analyze_additional_test_v2.py`.

`EVENT_SHAPE_SUMMARY_REPAIR.json` records a summary-only singleton-array
repair. MATLAB writes a single six-field event as a flat vector. The old
summary `len` counted its six fields; the corrected count is one event.
Independent native audits already used six-column event matrices and had
the correct counts. The `_V2` assessment summaries replace only these
event-count fields; all other metric, identity and byte fields are unchanged.
Earlier non-V2 summaries are retained for provenance.

`REPRODUCTION.md` gives input, execution, audit and delivery details.
`verify_association_summary_v2.py` independently recomputes the summary
arithmetic without tracker imports. The source/evidence archive and its
fresh-directory verification receipt are written under `output/` after
acceptance. Large native traces, raw point clouds and model weights remain
outside that compact archive, with their provenance and hashes retained.
