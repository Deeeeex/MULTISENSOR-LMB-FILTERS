# Nominal GCE origin diagnostic

This study returns to the original nominal GCE versus No-age failure after
the separate range-case intervention was completed. The two protocols and
freeze files distinguish the descriptive six-trajectory trace from the
four new native trajectories.

The trace (`trace.py`, `verify_trace.py`) locates the earliest meaningful
GCE/GS versus No-age difference and recounts all 2880 existing nominal
target robot frames. Gaussian decomposition uses the previously audited
moment formulas; the separate checker verifies native timing/counts and
the scalar decomposition arithmetic. It does not claim a second Gaussian
implementation or third-party replication.

The native runner changes only accepted extra negative scalar evidence at
frame 2. It preserves ordinary local negative evidence and the complete
GCE spatial distribution. `checkInitialNegative.m` checks exact original
reconstruction, the known log-odds change, positive-only identity, spatial
and metadata identity, and the empty case. Every native job runs it first.
`RUNNER_PATCH.json` and `AUDITOR_PATCH.json` preserve the bounded source
ports. `event_audit.py` independently computes the event from reconstructed
GCE terms while all other frames retain the complete original checks.

Execution order:

1. `run_stage.py nominal_origin_controls --workers 2`
2. `audit_stage.py nominal_origin_controls`
3. `run_stage.py nominal_origin_event --workers 2`
4. `audit_stage.py nominal_origin_event`
5. `finish.py`

Use `python3` for registration and native launching. Scientific scripts
use the existing `tmp/external_baselines/v2v_inference_venv/bin/python`.
Native jobs use MATLAB R2024a. Configurations and drivers refuse to
overwrite registered trajectories; the runtime receipts require a real
process exit, completion marker and exact expected file count.

Native logs are under `RUN/ICRA_NOMINAL_ORIGIN/<stage>/<condition>.log`.
Full native `.json.gz` records are retained in ignored `results/` paths
and bound by their hashes. `RESULTS_CN.md` reports both links and every
trajectory. `finish.py` independently recounts target matches and nearby
candidates from the saved native outputs before building the result and
final source manifest. The experiment ends without choosing a different
frame, duration or new algorithm from its outcomes.
