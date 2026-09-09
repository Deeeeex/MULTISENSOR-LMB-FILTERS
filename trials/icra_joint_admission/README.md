# Joint current-detection admission

Closed: the fixed-input primary failed two of its four continuation gates.
No new full-recursion run was started and no secondary was promoted.
The complete comparison is in [RESULTS_CN.md](RESULTS_CN.md); all 224
sequence/link/backend/rule rows are in [ALL_SCREEN_SCORES.csv](ALL_SCREEN_SCORES.csv).
Independent verification covered 3,235,260 distributions and 83,584 robot
frames. The raw-W warning received a separate elementwise arithmetic check
in [WEIGHT_ARITHMETIC_VERIFICATION.json](WEIGHT_ARITHMETIC_VERIFICATION.json).

The fixed primary is `r_plus * association_mass` for positive current-ratio
admission. A complete two-factor table also keeps the original mark gate,
its joint counterpart, and conditional no-mark admission. See
[PROTOCOL.md](PROTOCOL.md) for the precise formula, four continuation gates,
and the separate full-recursion requirements.

`SCREEN_FREEZE.json` binds all 56 previously completed source runs, their
14-segment roster, and 1,572 source/input/result hashes before new scores.
`screen.py` changes a single visited fusion input at a time and never feeds
the result back into tracking. `verify_screen.py` uses separate density
algebra, extraction and scoring. Generated tensors and output sets stay
under ignored `results/` for local reproducibility.

The execution order is frozen registration, `execute_screen.py`,
`verify_screen.py`, the recorded arithmetic check, then `build_report.py`
and `verify_archive.py`. The scripts refuse to overwrite
the recorded execution or final summaries. This is an exposed-data
diagnostic, not a held-out or full-recursion result.

```sh
tmp/external_baselines/v2v_inference_venv/bin/python -u trials/icra_joint_admission/execute_screen.py
tmp/external_baselines/v2v_inference_venv/bin/python -u trials/icra_joint_admission/verify_screen.py
tmp/external_baselines/v2v_inference_venv/bin/python trials/icra_joint_admission/verify_weight_arithmetic.py
tmp/external_baselines/v2v_inference_venv/bin/python trials/icra_joint_admission/build_report.py
tmp/external_baselines/v2v_inference_venv/bin/python trials/icra_joint_admission/verify_archive.py
```
