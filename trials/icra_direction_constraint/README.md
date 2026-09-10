# Pooled-evidence direction constraint

The primary follows the complete originally admitted ratio along one common
exponent multiplier. If the full correction reverses the current update
relative to the same weighted source predictions, it stops at the direction
boundary. The reference, numerical dead band, two directional controls,
cohort and continuation requirements are specified in [PROTOCOL.md](PROTOCOL.md).

All fourteen source segments are already exposed. The separate diagnostic
root trace is outside selection. The screen creates no recursive candidate
trajectory. Original distributions are preserved exactly wherever the
admitted exponents do not change. All fixed-input output sets and scores are
retained, including failed or unchanged groups.

The producer uses prediction/posterior moments and bracketed bisection.
The independent implementation reconstructs predictions from transmitted
ratios, removes the residual from the saved fused Gaussian and uses Brent's
method. It separately rebuilds the MAP cardinality distribution, extracts
sets and computes OSPA/GOSPA and their error components.

Execute with the existing scientific Python:

```sh
tmp/external_baselines/v2v_inference_venv/bin/python trials/icra_direction_constraint/register.py
tmp/external_baselines/v2v_inference_venv/bin/python -u trials/icra_direction_constraint/preflight.py
tmp/external_baselines/v2v_inference_venv/bin/python -u trials/icra_direction_constraint/execute_screen.py
tmp/external_baselines/v2v_inference_venv/bin/python -u trials/icra_direction_constraint/trace_reference.py
tmp/external_baselines/v2v_inference_venv/bin/python -u trials/icra_direction_constraint/verify_screen.py
tmp/external_baselines/v2v_inference_venv/bin/python trials/icra_direction_constraint/finish.py
```

Generated density arrays and output sets stay in ignored `results/`, with
their hashes included in the final archive. Logs and exit receipts preserve
actual execution. The final verification's `passed` flag concerns evidence
integrity; `advance_to_recursion` records the separate performance decision.

The inherited raw-association matrix multiplication previously emitted
NumPy runtime warnings despite finite, matching outputs. Its inputs have a
separate elementwise arithmetic audit. This screen preserves any warnings,
verifies that prior audit's exact inputs and counts, and does not claim that
the underlying numerical warning has been explained.
