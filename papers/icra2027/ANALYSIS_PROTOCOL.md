# Additional mechanism and outage analyses

This analysis plan is written before calculating its new comparisons. It
uses all 25 main sequences, the two existing communication conditions,
and the unchanged GCE, Scalar, and No-age KLA trajectories. It does not
select a new method or tune a parameter. Existing experiment outputs are
read-only; a portable snapshot records every input SHA-256.

## Communication phases

For a sequence of T scans, use the exact emulated outage boundaries
`a=floor(0.4*T)` and `b=floor(0.6*T)` in zero-based indices. Partition
all scans into before `[0,a)`, outage `[a,b)`, and after `[b,T)`.
Use the same partitions under reliable links as a matched scene-time
reference. Recompute position OSPA (p=2, c=12 m) from saved output sets
and truth, and check every value against the recorded metric. Average
over both receivers and scans within each sequence-phase, then weight
the 25 sequences equally. Retain every phase, method, and condition.
Report GCE-minus-reference differences with 10,000 paired complete-
sequence resamples (RNG seed 8301). These descriptive intervals are
not corrected for multiple comparisons. A phase effect is not a causal
estimate of communication loss because the trackers have different
histories; do not infer recovery time from phase averages.

## Fixed-input joint normalization

Use all delivered receiver-scans along the existing GCE trajectory.
Hold local inputs, label matching, supports, beta, admitted kappa, and
Gaussian curvature decisions fixed. Let z be the inherited log-odds
plus the admitted scalar increment; let (p0,I0) and (p1,I1) denote the
base and corrected spatial product/normalizer. Evaluate the complete
2 by 2 output substitution: spatial means from p0 or p1 and existence
logit z+log(I0) or z+log(I1). The p0/I0 cell is a guarded scalar
substitution, distinct from the recursively executed Scalar baseline.
The p1/I1 cell must reproduce the saved GCE output OSPA at every
delivered receiver-scan. Apply the existing pruning, MAP cardinality,
domain restriction, and OSPA scoring to each cell. No alternative is
fed into the next scan. Weight sequences equally and use the same
paired resamples. Present all four cells, including unfavorable ones.

## Manuscript interpretation

These analyses complement the already completed recursive component
ablations. Fixed-input substitutions identify immediate output effects
on GCE-visited states; phase comparisons describe when complete-track
gains occur. Neither replaces evaluation on new sequences or radio
traces. The core experiment data, method, and existing reported values
are not modified.
