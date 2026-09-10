# Nominal label genealogy and same-frame reimport

Read `RESULTS_CN.md`. This is a diagnosis of six already completed nominal
trajectories, not a new method or tracking run.

The census verifies prediction, pruning, source matching and exact population
balances, then propagates birth ancestry. Its independent verifier uses
ordinary sets instead of the producer's bitsets. The reimport extension joins
every remote-only retained output to the receiver's current local deletion
and records actual marginal probabilities versus missing-label bounds.

Valid execution order (completed files are not overwritten):

```sh
tmp/external_baselines/v2v_inference_venv/bin/python -u trials/icra_label_genealogy/execute_census_v2.py
tmp/external_baselines/v2v_inference_venv/bin/python -u trials/icra_label_genealogy/verify_census_v2.py
tmp/external_baselines/v2v_inference_venv/bin/python -u trials/icra_label_genealogy/reimport.py
tmp/external_baselines/v2v_inference_venv/bin/python -u trials/icra_label_genealogy/verify_reimport.py
tmp/external_baselines/v2v_inference_venv/bin/python trials/icra_label_genealogy/finish.py
```

`INTEGER_STORAGE_FIX.json` preserves the initial serializer failure and the
single label-type fix. Both original and repaired source freezes remain.
Large complete ancestry/event outputs are retained locally under `results/`
and bound by `FINAL_VERIFICATION.json`; source, tables and reports are tracked.
