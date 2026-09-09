# Reviewer revision experiments

The review, accepted questions and pre-outcome protocol are in
`SOURCE_REVIEW_CN.md` and `PROTOCOL.md`. Read `REVISION_LEDGER_CN.md` for
the completed evidence and remaining scientific limitations. This work
does not select a revised primary GCE from the new outcomes.

## Inputs and immutability

- `PRE_REVISION_FREEZE.json` identifies the original 1,347 algorithm sources,
  calibration, released detector and reviewed manuscript. The manuscript is
  revised; the algorithm, detector and calibration remain unchanged.
- `NEW_DATA_FREEZE.json` identifies all 3,028 new sensor files before
  inference. `NEW_DETECTION_MANIFEST.json` identifies the cached detections.
- `NEW_INPUT_MANIFEST.json` records the three prepared new tracking inputs
  and poses. `POSE_INPUTS_*.json` records the old-sequence pose conversion.
- `stages/*.json` defines every experiment before its outcomes. The mistaken
  `fixed_seen_transfer` registration was never executed; the correction and
  reason are retained in `CANCELLED_STAGES.json`.

Keep `fuseReviewerEvidence.m`, both replay runners, the input adapter and
registered sources unchanged. The `make_*` programs document the bounded
extensions and intentionally refuse to overwrite them. Pose converter
history is retained in `source_snapshots/` when a later converter version
was required to handle nonzero starting frame indices.

## Execution and checking

The experiment environment used Python 3.11 with NumPy/SciPy and MATLAB
R2024a. New detector inference used the released PointPillar checkpoint,
PyTorch 2.8.0, MPS float32, CPU voxelization and the author postprocessor.
Versions, numerical parity limits and public input provenance are recorded
in the manifests. No keys, anonymous download tokens or signed URLs are
saved in these reports.

In a separate reproduction checkout with the prepared inputs and empty
stage output/log directories, use the existing registered stage names:

```sh
python trials/icra_reviewer_revision/run_stage.py controls_development --workers 2
python trials/icra_reviewer_revision/audit_stage.py controls_development
python trials/icra_reviewer_revision/run_motion_stage.py motion_seen_transfer --workers 2
python trials/icra_reviewer_revision/audit_motion_stage.py motion_seen_transfer
```

Run these from the repository root.
The fixed-stage scripts refuse to overwrite an earlier execution. Each
MATLAB process runs all frames in its unit; truth enters scoring after
that arm's full tracking recursion. Runtime ledgers record its native
exit code and completion line. Logs live under `RUN/ICRA_REVIEWER_REVISION/`.

`audit_stage.py` and `audit_motion_stage.py` check complete saved inputs,
packet accounting, estimates and scores. Independent NumPy probability and
Gaussian reconstruction checks the admitted corrections. Identity and
old-output preflights distinguish unchanged recursion from new controls.
`audit_correlation_control_v2.py` independently reconstructs the known
joint-covariance control using a separate matrix path.

Once all stage audits pass, the summarizers create the recursive controls,
new-data and model-sensitivity reports and per-sequence CSVs.
`accept_revision_outputs.py` performs the final source/input/exit/file
integrity check and writes `REPLAY_ACCEPTANCE.json` and
`RESULT_FILES_MANIFEST.json`. Native file hashes identify retained outputs;
a fresh replay's runtime and gzip metadata can change those hashes even
when the numerical trajectories agree.

## Retained outputs and portable paper

Complete posterior traces remain under `results/`. They occupy several GiB
and are excluded from Git; they have not been deleted. The result manifest
and stage audits retain every SHA-256 identity. Raw public sensor files
remain in the ignored `tmp/external_baselines/v2v_official/` cache. Prepared
MAT inputs follow the repository's existing MAT ignore policy.

The paper directory's collector copies compact audited reports and complete
sequence-level values into `source_data/reviewer_revision/`. Its ZIP can
regenerate statistics, figures and the PDF without native posterior traces.
This is distinct from rerunning the full detector and tracker, which needs
the research checkout, input data and runtime described above.

The new segments contain only one previously absent original recording.
The primary result on that recording is unfavorable, and the correlated
Gaussian control exposes undercoverage despite PSD admission. Both are
retained in the paper and ledger; neither is removed through re-selection.
