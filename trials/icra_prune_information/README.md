# Current pruning information trial

The question is whether transmitting a current, locally pruned scalar can stop
the verified peer-retention path that defeated receiver-only refinement. This
isolated trial keeps the previous receiver-only family closed. See
[PROTOCOL.md](PROTOCOL.md) for the frozen rule, complete scope, and fair gate.

The first registration attempt stopped before writing stage configurations:
long historical result paths collided in MATLAB's 63-character struct-field
mapping. The corrected registration verifies those archived hashes through their
full V2 manifest and native reference records, while keeping executable and audit
source hashes in the existing native field format. No native trajectory started
before successful registration; the algorithm, fixtures and gate were unchanged.

Rebuild generated ports and fixture receipts in a fresh isolated copy, preserving
all referenced upstream artifacts. Every generator/registration/execution refuses
to overwrite its outputs. The bundled scientific Python environment is
`tmp/external_baselines/v2v_inference_venv/bin/python`; native execution uses
MATLAB R2024a. From the repository root, the sequence is:

```sh
tmp/external_baselines/v2v_inference_venv/bin/python trials/icra_prune_information/make_ports.py
tmp/external_baselines/v2v_inference_venv/bin/python trials/icra_prune_information/make_audit.py
tmp/external_baselines/v2v_inference_venv/bin/python trials/icra_prune_information/preflight_fixture.py
tmp/external_baselines/v2v_inference_venv/bin/python trials/icra_prune_information/register.py
tmp/external_baselines/v2v_inference_venv/bin/python trials/icra_prune_information/run_stage.py prune_info_controls
tmp/external_baselines/v2v_inference_venv/bin/python trials/icra_prune_information/audit_stage.py prune_info_controls
tmp/external_baselines/v2v_inference_venv/bin/python trials/icra_prune_information/run_stage.py prune_info_shared
tmp/external_baselines/v2v_inference_venv/bin/python trials/icra_prune_information/audit_stage.py prune_info_shared
tmp/external_baselines/v2v_inference_venv/bin/python trials/icra_prune_information/finish.py
```

The native wrapper launches one job per link, runs three backends serially in
each job, and records process exits and expected output counts. Complete lane
logs are under `RUN/ICRA_PRUNE_INFORMATION/<stage>/`. Configurations and source
hashes are in `FREEZE.json` and `stages/`. Native payloads and detailed reimport
and reentry events are kept locally under `results/` and hash-bound by the audits
and final verification; compressed high-volume artifacts are excluded from Git.
`RESULTS_CN.md`, `RESULTS.json` and `ALL_SCORES.csv` are the final paired report.

The complete new record has 24 columns:

`frame, receiver, birth_frame, birth_location, proposal_r, actual_r,
missing_input_index, reported_r, reported_pD, old_r_1, old_r_2,
b_1, b_2, q_1, q_2, old_beta_1, old_beta_2, actual_beta_1, actual_beta_2,
old_age, actual_age, spatial_log_normalizer, old_inherited_logit,
actual_inherited_logit`.

The old 60-column proposal record retains its original operands and weights.
Only its actual-r columns 7 and 10 change. Neither the sender trailer nor the
new fusion hook has access to annotation IDs or truth; those enter only after
the complete native trajectory, during reporting and independent diagnosis.
