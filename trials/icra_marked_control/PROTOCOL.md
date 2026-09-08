# Shared-information conservative-recency control

This secondary ablation is specified after partial results from the registered
25-sequence experiment exist, and before any full-cohort aggregate or any
result from this extra arm. It is not a newly registered primary method and
does not replace M-ECR-S or its original references in METHOD_FREEZE.json.

Run exactly one additional control on all nine development sequences and all
25 reserved fusion-selection sequences, under both existing radio conditions.
Use the identical marked local likelihood update, already fitted calibration,
inputs, dynamics, birth, label matching, spatial fusion, pruning and extraction.
The development inputs retain their sequence-excluded calibration; the reserve
uses the pre-existing full-development calibration. No parameter is fitted.

For the same fusion inputs, set rCR=min(r0,rER). This is the c=0 limit of ECR;
it retains negative recency corrections and removes all positive corrections.
Use the already implemented conservative fusion function from the earlier
unmarked experiment. The native Gaussian packet uses 208 bytes per Bernoulli
plus its 32-byte header, with no evidence-ceiling scalar needed by the rule.

The purpose is to measure the contribution of admitting supported positive
corrections beyond conservative truncation when the local observation model
is shared. The existing score-free synthetic CR and ECR-A results remain the
matching no-score fallback comparison; they need not be rerun.

Before the new full runs, execute marked ER and this control on development
sequence 0000 in a dedicated preflight directory. Require ER states, labels,
metrics, diagnostics and packet accounting to match the completed common-port
preflight, with only runtime and provenance fields allowed to differ. Check
the control's existence equation independently and use existing analytic CR
fixtures. Freeze the extra source manifest before tracking the full cohorts.

Each sequence runs in its own MATLAB single-computation-thread process, at
most two additional processes concurrently. Preserve all completed results
and require successful exit, a completion line, and both radio result files.
The original three-process experiment continues with its original sources.

For the extra comparison use all original units, sequence-macro OSPA with
cutoff 12 m and order 2, missed/false GOSPA squared costs, common matched truth
support, and actual packet lengths. Report paired descriptive 95% sequence
bootstrap intervals, 10000 resamples and seed 8301. Evaluate the original
primary against this additional control; do not promote the control to a new
primary on the same reserve. The detector-training and route-dependence
limitations of the original cohort still apply.
