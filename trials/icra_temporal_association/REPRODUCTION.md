# Replay and evidence reproduction

The study starts from the unchanged `dc6a4c14` checkout. It adds only
`trials/icra_temporal_association`, `trials/icra_association_test` and its
own `RUN/ICRA_TEMPORAL_ASSOCIATION` logs. All 1,347 original protected source
files retain their recorded hashes. The original manuscript is unchanged.

## Fixed implementation

The selected version is `marked_gaussian_evidence_assoc_quality_nis`.
`runScreenedAssociation.m` is the final native entry point. The corresponding
independent auditor is `audit_screen_assessment_v2.py`, including the original
No-age KLA reference. This auditor is also valid for the completed QN exposed
assessment stages. `CANDIDATES_V3.md` records the prospective selection rule;
`PERSISTENT_METHOD_SPEC.md` records the persistent branch state and abstention
equations. Parameter selection ends before the additional test acquisition.

Keep the existing MATLAB R2024a runtime and the scientific Python environment
at `tmp/external_baselines/v2v_inference_venv/bin/python`. The detector uses
the pinned released checkpoint and MPS float32; its seed is registered per
frame and source. No training or recalibration is performed.

## Additional input pipeline

The cohort is all 14 prospectively registered mobile-pair segments, 2,172
paired frames from five collection dates. `icra_association_test/PROTOCOL.md`
defines the primary and secondary evaluations. The order is:

1. `fetch_inputs.py`: selected-method gate, cohort freeze, public archive
   ranges, per-file CRC/SHA-256 and exact-cloud overlap check.
2. `check_adapter.py`: raw bytes, official pose parity, coordinate closure,
   class/ID rules and cloud representation. `ADAPTER_CHECK_ROUNDOFF_REPAIR.json`
   records the sole self-identity tolerance correction from `1e-13` to
   `1e-12`; all 2,172 pairs also agree with a separately formed relative
   transform within `2.54e-13`. The coordinate conversion is unchanged.
3. `freeze_inference.py`, then `infer_transfer.py`: immutable dependencies,
   unchanged detector and compact source detections.
4. `prepare_inputs.py`, then `audit_inputs.py`: saved measurements, score
   ratios, poses and scoring-only labels, independently checked against
   raw inputs and detector outputs.

The original interrupted acquisition and resumed acquisition logs are both
retained. The resumed downloader changes queue management and failure
logging only. Public archive access tokens and signed URLs are not stored
in these study artifacts.

## Native execution and analysis

Registration consumes completed input checks. In a fresh checkout with the
frozen compact input files and original dependencies present, the commands
for the additional cohort are:

```sh
python3 trials/icra_temporal_association/run_screened_stage.py association_screen_selected_test --workers 2
tmp/external_baselines/v2v_inference_venv/bin/python trials/icra_temporal_association/audit_screen_assessment_v2.py association_screen_selected_test
tmp/external_baselines/v2v_inference_venv/bin/python trials/icra_temporal_association/analyze_additional_test_v2.py
```

The registered stage has 14 units, five arms and two delivery conditions:
140 separate native results. Every method executes its own full recursion
on the same saved measurements and radio draws. The runner refuses to
overwrite any prior stage. A repeat therefore needs a fresh result/log
location with the frozen source tree; do not remove prior results to rerun.

The additional test used the same v2 audit body scheduled per completed
unit through `audit_screen_assessment_unit.py`. `UNIT_AUDIT_SCHEDULING_PATCH.json`
records every wrapper-only replacement; `UNIT_AUDIT_PARITY.json` verifies
exact equality of the benchmark's physical checks, diagnostics, scores,
inputs and parity records. No tracking or audit equation changed. Unit
selection follows native completion status and never a score criterion.
Each report under `partial_audits/` contains its immutable completed-unit
record and runtime snapshot. `merge_unit_assessment_audits.py` creates the
final full-stage audit only after all 14 units have exited successfully,
all 140 files exist, and every unit's results and audit-source hashes match.
The single full-stage v2 command above remains an equivalent fresh-replay
audit path.

For earlier stages use their exact registered JSON and matching runner:
instrumentation, restored D/T/R/S, the column-corrected S assessment, or
screened Q/N/QN. `REPLAY_ACCEPTANCE.json` lists all valid stages. The original
50 domain-shadowed outputs and 35 failed zero-output units are preserved
separately and excluded from result comparisons.

`accept_association_results_v2.py` checks source/input/result hashes, actual
native exits, output counts, audit receipts, prospective ordering and final
analysis integrity. `RESULT_FILES_MANIFEST.json` lists valid and invalid
native archives separately. `verify_association_summary_v2.py` independently
recomputes means, scored identity rates, grouped differences, bootstrap
intervals and geometric summaries without importing the tracker or its
analysis helpers.

`EVENT_SHAPE_SUMMARY_REPAIR.json` records the corrected summary reader for
zero, one and multiple branch-event rows. A single six-field MATLAB row
is a flat JSON vector; it counts as one event. The `_V2` exposed summaries
use the independent audits' existing event counts. All OSPA, GOSPA,
cardinality, identity and byte fields remain exactly unchanged. The initial
analysis exception and its original source are retained. The native stage
itself and its completed audits required no rerun or method change.

## Compact delivery

`package_association_study_v2.py` creates the source/evidence archive under
`output/` after acceptance. It includes the two study directories, original
logs and compact frozen detection/score/pose inputs. Large native traces,
raw point clouds and the detector checkpoint stay outside the archive;
their hashes and acquisition/provenance records are retained.

The archive is checked member by member, extracted to a fresh directory,
and the independent summary verifier runs there. `output/PORTABLE_REBUILD.json`
records the exact archive hash, file count and successful command. This
verifies portable summary reproduction; it does not claim that packaging
reran the full MATLAB study.
