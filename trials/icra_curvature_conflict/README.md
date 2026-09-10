# Curvature conflict screen

`PROTOCOL.md` freezes one primary and two directional controls. No candidate
tracking recursion is run here. `RESULTS_CN.md` is the outcome entry point.

The first launch failed before any completed output because unchanged
missing-source fields contain NaN. `MISSING_VALUE_CHECK_FIX.json` binds the
sole assertion repair and preserves the initial freeze, source, and failure.
Use the v2 execution, freeze and verifier for the completed screen.

Successful execution order (outputs cannot be overwritten):

```sh
tmp/external_baselines/v2v_inference_venv/bin/python -u trials/icra_curvature_conflict/execute_screen_v2.py
tmp/external_baselines/v2v_inference_venv/bin/python -u trials/icra_curvature_conflict/verify_screen_v2.py
tmp/external_baselines/v2v_inference_venv/bin/python trials/icra_curvature_conflict/finish_screen.py
```

`SCREEN_FREEZE_V2.json` binds all inputs and pre-outcome source. The final
verification additionally binds the exact report builder, every saved output,
the independent density/score checks and the original failed execution.
