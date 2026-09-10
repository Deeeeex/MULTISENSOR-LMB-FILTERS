# Known local censor: one complete recursive causal case

`PROTOCOL.md` fixes the intervention, all twelve trajectories, both links,
all three methods, and the sole gate for expanding the same rule. The source
and fixture receipts in `FREEZE.json` precede native control execution.
This directory is isolated from the core filter and manuscript.

## Why a threshold-valued censor can sustain a reimport

Consider an already qualified missing self source and one represented peer
label. The peer was retained and transmitted only if r_peer > tau=0.001.
There is just one represented Gaussian, so its normalized spatial pool has
log integral I=0. The absent self source admits no extra current Gaussian
or scalar exponent. With strictly positive existence weights beta_self
and beta_peer summing to one, the original fused log-odds is

    L_old = beta_self * logit(tau) + beta_peer * logit(r_peer).

Thus L_old > logit(tau), and the output again survives r>tau pruning. The
same argument holds for either b or q selected by GCE's inherited-history
rule, and for No-age's b. This is a property of the common censor policy,
not a GCE-specific property. It does not prove every absent label has
observable support, I=0 under other spatial pools, or the existence of a
physical target. Numerical guards still apply in the implementation.

For equal weights and a known local r_local<=tau, the refined output
survives exactly when

    odds(r_local) * odds(r_peer) > odds(tau)^2

in exact arithmetic with I=0, taking both r operands after the unchanged
logit clipping to [1e-9,1-1e-9]. Using the known local value can therefore
allow a reimport to be pruned. The complete native intervention determines
whether this helps target detection and full-set error after recursion.
The common policy and the remaining GCE versus No-age gap must be assessed
separately in the final results.

## Reproduction and records

Run commands from `/Users/dex/Desktop/Code/icra27` with the frozen Python
3.11 runtime `tmp/external_baselines/v2v_inference_venv/bin/python` and
MATLAB R2024a. Native sources and auditors are immutable after registration.
The stage scripts refuse to overwrite any registered outputs.

1. `make_runner.py` and `make_auditors.py` produce exact source-port receipts.
2. `preflight_fixture.py` records the native fixture exit and log.
3. `register.py` freezes both stages together.
4. `run_stage.py known_censor_controls`, then the preserved v1 audit.
   The v1 audit completed every trajectory check but failed during its final
   JSON write on a NumPy integer; its three CSVs and six event files remain.
5. `register_audit_v2.py` freezes the storage-only correction and binds the
   six completed controls and all partial audit outputs. The native runner,
   formulas, tolerances, configurations and gate stay frozen at v1.
6. `audit_stage_v2.py known_censor_controls` produces a complete audit under
   new filenames; `run_stage_v2.py known_censor_refined` requires that audit.
7. `audit_stage_v2.py known_censor_refined`, then `finish_v2.py`, complete the
   independent target/reimport recount and apply the same expansion gate.

Logs are under `RUN/ICRA_KNOWN_CENSOR/<stage>/<condition>.log`. The final
entry points are `RESULTS_CN.md`, `RESULTS.json`, `ALL_SCORES.csv` and
`FINAL_VERIFICATION.json` once successfully produced. The two audit files
contain full density and recursion checks; the event CSVs and native JSON
files are compressed under `results/`. The storage correction and exact
ports are recorded in `AUDIT_STORAGE_FIX.json` and `AUDIT_FREEZE_V2.json`.
Extra event columns are documented
in `PROTOCOL.md`; the old proposal diagnostics are never silently presented
as actual refined operands.
