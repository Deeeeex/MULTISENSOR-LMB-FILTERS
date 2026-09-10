# Peer support for extra negative evidence

The single primary scales the original extra negative exponent by
`1 - r_peer * a_peer`, using only current, represented, positive-weight
source support. The positive gate and ordinary local missed-detection
update remain as recorded. The exact scope and stop rules are in
[PROTOCOL.md](PROTOCOL.md).

This directory first completes a fixed-input screen on all 56 source runs
from the frozen 14-segment cohort. It does not run a recursive candidate.
No-age and full physical packet accounting become required comparisons
only if every prespecified screening comparison passes. Four rule outputs,
both source backends and all sequence rows are retained. Secondary rules
cannot replace a failed primary.

`preflight.py` checks algebra and exact original parity on one input from
each backend without evaluating an alternate rule. `register.py` then
freezes code, cohort, prior results and all twelve inputs to the separate
exploratory label trace. `execute_screen.py` retains the actual child-process
exit and log. `verify_screen.py` independently rebuilds the density from
the transmitted ratios and uses separate extraction and scoring code.

`trace_motivation.py` follows the already exposed label `(3, 100004)` through
all 240 frames and both robots in the original, receiver-only and shared
pruning-information contexts. It substitutes only that label's existence
at each fixed input, checks actual output sets and target-detection totals,
and never feeds those substitutions back. This trace is outside selection.

Run in this order with the bundled experiment Python:

```sh
tmp/external_baselines/v2v_inference_venv/bin/python trials/icra_peer_detection/preflight.py
tmp/external_baselines/v2v_inference_venv/bin/python trials/icra_peer_detection/register.py
tmp/external_baselines/v2v_inference_venv/bin/python -u trials/icra_peer_detection/execute_screen.py
tmp/external_baselines/v2v_inference_venv/bin/python -u trials/icra_peer_detection/trace_motivation.py
tmp/external_baselines/v2v_inference_venv/bin/python -u trials/icra_peer_detection/verify_screen.py
tmp/external_baselines/v2v_inference_venv/bin/python trials/icra_peer_detection/finish.py
```

Scripts refuse to overwrite recorded summaries or execution receipts.
Detailed arrays and output sets remain in ignored `results/`, bound by
SHA-256 in the final verification. Current results belong in the generated
`RESULTS_CN.md` and `FINAL_VERIFICATION.json`; the latter distinguishes
successful verification from successful performance gates.

An initial registration attempt stopped before writing a freeze or running
any alternate: four older original trace inputs are referenced by the prior
verified stage configuration, not directly listed in its final manifest.
Registration now validates that configuration against the prior manifest,
then validates its four explicit native hashes. The eight newer native
trace inputs are checked directly. This was a pre-execution provenance-path
correction; the rule, fixture results, cohort and gates did not change.
