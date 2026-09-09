# Persistent missed-evidence experiment

The method and selection contract is in `PROTOCOL.md`. All original study
files are protected by stage hashes. New methods alter only the existing
negative-support message value and run their own complete recursion.

**Closed: the frozen screen failed.** See `RESULTS_CN.md` for all nine V2V
and five V2X screen sequences, the repaired mechanism case, and the added
false-output counterevidence. Do not extend or tune this rule after failure.

- Native implementation: `missHistorySupport.m`, `runMissHistoryReplay.m`.
- Source-copy receipt: `RUNNER_PATCH.json`; numerical fixtures: `checkMissHistory.m`.
- Frozen stages: `stages/miss_history_preflight.json`, `stages/miss_history_screen.json`.
- Execution: `python3 run_stage.py STAGE --workers 2` from this directory,
  or the repository-relative equivalent from the repository root.
- Verification: scientific Python `audit_stage_v4.py STAGE`.
- Canonical summary: scientific Python `summarize_v2.py --preflight-only`,
  then `summarize_v2.py` after the complete screen audit.

`summarize.py` is preserved as the initially frozen version. Before the
mechanism case completed, code inspection found that its frame-119 counts
would incorrectly use an empty fusion log during the communication blackout.
`SUMMARY_POOL_REPAIR.json` records the diagnostic-only fix to use the actual
local Gaussian and existence records on those frames. No method, score or
selection rule changes. Never overwrite a completed native stage or analysis.

`EMPTY_WEIGHTS_AUDIT_REPAIR.json` preserves the first auditor's failure on
an empty native association-weight log. The v2 auditor explicitly checks
zero positive support and zero association mass in that case. The native
algorithm, frozen source files, and selection criteria are unchanged.

`ROW_SCHEMA_REPAIR.json` records the v3 row-assembly fix: reuse and verify the
wire-byte metric already returned by the existing scorer. Native runs are
retained without rerunning.

The completed preflight also receives an explicit bounded elementwise
positive-mark recheck after matrix-product runtime warnings. See
`BOUNDED_POSITIVE_SUM_CHECK.json` and `POSITIVE_MARK_RECHECK.json`; the v4
auditor uses those finite factor and sum checks for subsequent stages.

Post-outcome diagnostics are in `FALSE_SUPPORT_DIAGNOSTIC.json` and
`FALSE_EXTRA_POLARITY_DIAGNOSTIC.json`; `verify_false_polarity.py` reproduces
the latter exactly. These describe the candidate's visited inputs, not the
recursive performance of another proposed gate. `SCREEN_VERIFICATION.json`
is the initial full verification; `FINAL_VERIFICATION.json` also binds the
final report and added diagnostics.
