# Source visibility and range diagnosis

See `RESULTS_CN.md` for the verified findings. This directory contains no new
tracking run, pose correction, or manuscript change. All cases were already
exposed. The previous miss-history study is closed at commit `82ff5d95`.

Written protocols:

- `PROTOCOL.md`: fixed detector-box and raw-signal diagnosis.
- `RANGE_RECALL_PROTOCOL.md`: the single recording-held-out calibration model,
  fixed before fitting; no tracking outcomes or V2X fit data.

Completed entry points (each preserves existing outputs rather than replacing
a completed analysis):

- `diagnose_boxes.py` → `BOX_VISIBILITY_DIAGNOSTIC.json`.
- `inspect_raw_samples.py` → `RAW_SAMPLE_INSPECTION.json`, `raw_samples.png`.
- `recheck_raw_geometry.py` → `RAW_GEOMETRY_RECHECK.json`.
- `check_alignment_sequence.py` → `RAW_ALIGNMENT_SEQUENCE.json`.
- `calibrate_range_recall.py` → `RANGE_RECALL_ROWS.csv`, `RANGE_RECALL_CALIBRATION.json`.
- `verify_diagnostic.py` → `DIAGNOSTIC_VERIFICATION.json`.
- `build_report.py` → `RESULTS_CN.md`, `REPORT_BUILD.json`.

Analysis Python is `tmp/external_baselines/v2v_inference_venv/bin/python` from
the repository root. The raw plotting script additionally appends the
existing `tmp/external_baselines/venv/lib/python3.11/site-packages` for
Matplotlib; its own directory must also be on `sys.path` when run via `runpy`.
No packages were installed or changed. Source and raw-file hashes are kept
with the respective reports; the sequence raw checks also verify archive CRC.

The initial raw plotting calculation emitted BLAS runtime warnings. Explicit
finite coordinate sums reproduce all four raw and prepared point counts;
`RAW_GEOMETRY_RECHECK.json` preserves this validation without claiming to know
the warning's cause. The coarse registration checks are diagnostics only and
do not certify a replacement pose or identify the physical cause of the offset.
