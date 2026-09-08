# Reproducing the method iteration

Run from the repository root in an isolated checkout. The saved compact MAT
inputs, original paired simulation caches and complete compressed outputs
are tracked. Python needs NumPy, SciPy and Matplotlib. MATLAB R2024a was used
with one computation thread. The local Python interpreter used for auditing
is `tmp/external_baselines/venv/bin/python`.

Fetch the original pinned author dependencies with
`python3 trials/icra_external_fusion/fetch_dependencies.py`. This checks out
the TC and DMSTrack dependencies under the ignored `tmp/external_baselines/`.
The audit verifies original raw-source hashes there. Regenerating compact
transfer inputs is optional: `fetch_transfer_transforms.py`,
`audit_transfer_overlap.py`, then `prepare_transfer.py`. The fetch reads only
the selected coordinate transform arrays from the public archive; saved
CRC32/SHA-256 metadata remains the provenance reference.

Do not run `make_*.py` or `freeze_sources.py` as part of reproduction. They
are the historical construction steps. A changed source hash is an error,
not permission to replace an already registered snapshot.

## Complete tracking runs

These commands overwrite the corresponding outputs, so use an isolated
checkout. Original baseline outputs are reused and remain the reference.
Set the MATLAB executable for another installation if necessary.

```sh
mkdir -p RUN/ICRA_ITERATION
set -o pipefail
/Applications/MATLAB_R2024a.app/bin/matlab -singleCompThread -batch "addpath('trials/icra_method_iteration'); runMethodReplayFast(0,8,false);" 2>&1 | tee RUN/ICRA_ITERATION/ir_full_accelerated.log
/Applications/MATLAB_R2024a.app/bin/matlab -singleCompThread -batch "addpath('trials/icra_method_iteration'); runConservativeReplayFast(0,8,false);" 2>&1 | tee RUN/ICRA_ITERATION/cr_full_accelerated.log
/Applications/MATLAB_R2024a.app/bin/matlab -singleCompThread -batch "addpath('trials/icra_method_iteration'); runConfirmedReplayFast(0,8,false);" 2>&1 | tee RUN/ICRA_ITERATION/cgr_full_development.log
/Applications/MATLAB_R2024a.app/bin/matlab -singleCompThread -batch "addpath('trials/icra_method_iteration'); runConservativeCases(2901,2920);" 2>&1 | tee RUN/ICRA_ITERATION/cr_original_cases.log
/Applications/MATLAB_R2024a.app/bin/matlab -singleCompThread -batch "addpath('trials/icra_method_iteration'); runConfirmedCases(2901,2920);" 2>&1 | tee RUN/ICRA_ITERATION/cgr_original_cases.log
/Applications/MATLAB_R2024a.app/bin/matlab -singleCompThread -batch "addpath('trials/icra_method_iteration'); runTransferReplay(0,6,false);" 2>&1 | tee RUN/ICRA_ITERATION/reserved_transfer_full.log
```

The last runner's indices 0--6 select the seven IDs in the fixed manifest;
they do not mean original sequence IDs 0000 through 0006. The last console
completion line prints these indices. All result files identify the actual
sequence. Sequence 0000 is the separately reported overlap control.

For the final run, monitor with
`tail -f RUN/ICRA_ITERATION/reserved_transfer_full.log`.

After tracking, write the new audit summaries into a separate directory:

```sh
python3 trials/icra_method_iteration/analyze_development.py --candidate ir --accelerated --output-dir tmp/iteration_reproduction
python3 trials/icra_method_iteration/analyze_development.py --candidate cr --accelerated --output-dir tmp/iteration_reproduction
python3 trials/icra_method_iteration/analyze_development.py --candidate cgr --accelerated --output-dir tmp/iteration_reproduction
python3 trials/icra_method_iteration/analyze_cases.py --candidate cr --output-dir tmp/iteration_reproduction
python3 trials/icra_method_iteration/analyze_cases.py --candidate cgr --output-dir tmp/iteration_reproduction
python3 trials/icra_method_iteration/analyze_transfer.py --output-dir tmp/iteration_reproduction
```

Compare the new `aggregate`, `paired` and per-run metric fields against the
registered summaries. Runtime and compressed-byte hashes can change
between executions; state outputs and metrics should reproduce. Do not
conflate compressed-file hash changes from a new run with changed method
source. The pre-transfer source snapshot includes the two saved complete
development summaries; `--output-dir` keeps those registered files intact.
The default analysis commands in `README_CN.md` re-audit the saved original
outputs and reconstruct the published iteration report.

## Canonical outputs and excluded diagnostics

| Directory | Scope |
| --- | --- |
| `results_v2v_fast/` | Complete nine-sequence IR and original-ER parity replay |
| `results_v2v_conservative_fast/` | Complete nine-sequence CR replay |
| `results_v2v_confirmed_fast/` | Complete nine-sequence CGR replay |
| `results_cases/` | Complete 60-case CR replay |
| `results_cases_confirmed/` | Complete 60-case CGR replay |
| `results_transfer/` | Complete seven selected sequences, eight arms, two conditions |
| `results_v2v/`, `results_v2v_conservative/` | Partial slow development diagnostics, 0000--0003 only |

The `*_first3.*` and `*_first4.*` summaries are dated development preflights,
not full-cohort results. Two slow dense-sequence runs and one original
runtime diagnostic were interrupted; their logs are retained. Smoke/profile
artifacts under ignored `tmp/` are not evidence for the reported comparisons.
The empty IR packet and Hungarian coverage-vector orientation issues were
fixed during preflight, before their complete frozen runs.

Synthetic containers hold truth for scoring local/post-fusion outputs, but
the scores do not feed back into prediction, update or fusion. The real
`runDensity` tracker function has no truth argument; the outer runner scores
its completed output afterward.
