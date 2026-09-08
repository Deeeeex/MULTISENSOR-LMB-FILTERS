# Selective current innovations: fixed development experiment

The complete absolute-ceiling experiment, conservative control, tempered
recency experiment, missing-label diagnostic, and marked joint-innovation
experiment were inspected before designing this round. Full accumulation of
marked local innovations reduced misses but increased false-target cost and
failed the complete nine-sequence development gate. All 34 real trajectories
are now seen development or transfer evidence. No reserved-test claim applies.

## Fixed construction

All methods use the unchanged marked local LMB update, calibrated mark
likelihood ratios, dynamics, association, label alignment, geometry, spatial
fusion, cropping, pruning and MAP extraction. For local source j compute

    delta_j = logit(r_j_after_local_update) - logit(r_j_predicted)
    v_m = max(0, (L_m - 1) / (L_m + 1))
    g_j = sum_m W_jm v_m.

Here L_m is the existing target-versus-clutter mark likelihood ratio, and W
is the unchanged association distribution including a missed-detection branch
whose contribution to g is zero. The helper normalizes finite nonnegative W
exactly as in the preceding support experiment. L=1 has no mark evidence; no
new fitted threshold or calibration is introduced. This gate is a bounded
mark-discrimination heuristic, not a confidence interval or a second likelihood.
g is reset to zero without a current direct observation opportunity.

Let b be eligible base weights, q the unchanged age weights, x the local
posterior logits and eta the spatial normalizer. Define

    age = sum_j (q_j - b_j) x_j
    beta = q if age < -1e-12, otherwise b
    boost = sum_j (1 - beta_j) g_j max(delta_j, 0)
    z = sum_j beta_j x_j + log(eta) + boost.

Boost is used only when at least two eligible positive-weight sources both
represent the aligned label. Otherwise boost is zero. Untouched exclusions
and observable absence censors are unchanged. The 1e-12 log-odds tie band
fixes the ambiguous beta at numerically equal pooled posteriors; it is a
numerical convention selected before preflight, not a tuned threshold. The
zero-boost rule agrees with conservative recency within numerical tolerance.
The original CR reference uses its exact original implementation.

The fixed primary is `marked_selective` (M-SI). Three fixed ablations are:

- `marked_selective_no_history`: beta=b for every label.
- `marked_selective_no_mark`: v=1 for every detection, so g is the detection
  association mass; the marked local likelihood update remains shared.
- `marked_selective_signed`: use delta instead of max(delta,0) in boost.

No secondary can replace the primary based on these outputs. The construction
tests whether discriminative current support can admit useful positive
innovations while retaining a conservative inherited posterior. It may still
amplify correlated false detections; this is a hypothesis, not a guarantee.
Ordinary prior/likelihood separation is an existing idea, not claimed novelty.

Each source sends its own current delta and g, totaling 224 B per Bernoulli
plus a 32 B header. Values are serialized and decoded, not read from another
source's private state. Fusion clears g; the next local update recomputes both
scalars. Original decoded label IDs are logged before alignment solely for
independent checking; they already exist in the packet and add no wire field.

## Execution and decision

Before any trajectory, check algebraic endpoints, negative and stale
innovations, neutral and informative marks, exclusion, censoring, unchanged
spatial densities and exact packet round trips. Run preflight on development
0000 in both radio conditions with all four candidates and the original M-CR.
Require exact CR trajectory/metric/packet parity except runtime and the
additional diagnostic fields; compare its original 26 record columns exactly.
Independently recompute local deltas and join every represented fusion input
back to its source, time and original label to verify the transmitted values.

Then run all nine development sequences, both radio conditions, all four
candidates. Reuse and independently rescore the five complete shared controls
M-No-age, M-ER, M-CR, M-ECR-C and M-ECR-S. Retain the preceding marked JE and
JE-R results for the development comparison. No selective sequence expansion.

Continue to all 25 seen-transfer sequences only if the fixed primary has lower
sequence-macro OSPA than M-No-age and M-ER in both conditions, and no higher
OSPA than M-CR and M-ECR-S in both. This stronger continuation rule is selected
before preflight because a useful candidate must improve on the best previous
main candidate. On failure, retain all nine-sequence results and stop this
candidate's expansion. A secondary cannot be promoted using this same cohort.

OSPA: position only, p=2, cutoff 12 m. Report all sequence values, paired
10,000-sequence descriptive bootstrap intervals, missed/false GOSPA squared
costs, localization on common matched truth and actual native packet costs.
Sensor-frame observations are not independent replicates. Every sequence
requires successful process exit, the completion marker and eight output
files. Register source before preflight and the complete stage after preflight.
Retain previous source, inputs, negative outcomes and the manuscript.
