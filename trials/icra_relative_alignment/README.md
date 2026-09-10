# Relative spatial alignment experiment

This trial tests a new common source-coordinate model, using current raw
LiDAR occupancy. It keeps the original GCE and No-age rules and applies
the same estimated translations to each. See `PROTOCOL.md` for the frozen
rule, complete cohort, stop gate, additional communication phase and limits.

From the repository root, use
`tmp/external_baselines/v2v_inference_venv/bin/python` for the Python scripts.

1. `build_runner.py`, then `fixtures.py`, then `register.py`.
2. `execute_features.py` acquires/reads the frozen raw entries and constructs
   compact, independently checked packet grids. Its complete log is
   `RUN/ICRA_RELATIVE_ALIGNMENT/features.log`.
3. `register_stage.py parity`, `run_stage.py parity`,
   `audit_stage.py parity`. This can run while raw features are prepared.
4. After raw preparation exits successfully, `estimate.py` checks every
   candidate integer overlap and saves the current-frame translations.
5. `register_stage.py corrected`, `run_stage.py corrected`,
   `audit_stage.py corrected`, then `finish.py`.

Native MATLAB R2024a jobs use at most two processes, retain per-sequence
logs and record actual exit codes. Completed outcomes are never overwritten.
Missing public raw entries are streamed in bounded ranges, CRC checked and
converted into compact grid shards. The unchanged raw cache is preserved;
no large raw download is retained just to duplicate the public archive.
Source hashes and the complete feature journal bind any resumed input work.

The new phase sends two 4,032-byte packets per frame. Both receivers solve
the same registration after receiving those current packets. The report
includes their extra 33,024 wire bytes per frame and measured computation.
No intermittent-link, real-time, novelty, or unexposed-recording claim follows
from this first reliable-link experiment.
