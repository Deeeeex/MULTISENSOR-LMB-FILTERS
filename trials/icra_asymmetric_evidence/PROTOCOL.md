# Branch-supported positive and negative current evidence

This round follows the complete selective experiment and complete fixed-input
negative-evidence diagnostic. Selective positive innovations improved all
four primary No-age/ER interval comparisons but failed the stronger expansion
gate against intermittent M-ECR-S. The positive-only restriction was not
supported by its signed ablation. Neither previous arm is promoted.

The new hypothesis uses different observable support for positive and negative
evidence. Positive evidence is supported by informative detection marks;
negative evidence is supported by the missed-detection association branch
and actual current sensing opportunity. All trajectories are already seen
development/transfer data; all preceding outcomes are retained.

## Fixed rule and controls

Retain the unchanged marked local update. Let W include its missed branch,
L be the existing mark likelihood ratios, a be detection association mass,
and pD be the exact nominal detection probability evaluated at the predicted
track in the existing local update. Compute

    delta = logit(r_after) - logit(r_predicted)
    g_positive = sum_m W_m max(0, (L_m-1)/(L_m+1))
    g_negative = (1-a) pD/(2-pD).

The missed branch includes empty measurements (a=0). pD is 0.9 inside the
unchanged sensing domain and zero without current direct opportunity. The
negative factor pD/(2-pD) is (1-L_miss)/(1+L_miss) for the Bernoulli missed
likelihood L_miss=1-pD. Neither a nor a positive detection mark is treated as
proof of a true target. Both gates are bounded branch-support heuristics,
not calibrated error probabilities or extra independent likelihood factors.
No threshold, calibration, decay or tracking parameter is fitted here.

Use exactly the preceding eligible weights b, age weights q, and spatial
normalizer eta. beta=q when sum(q-b)logit(r)<-1e-12, otherwise beta=b. The
primary `marked_asymmetric` (M-AE) has

    z = sum beta_j logit(r_j) + log(eta)
        + sum_j (1-beta_j) [g_positive_j max(delta_j,0)
                           + g_negative_j min(delta_j,0)].

The added sum is used only when at least two eligible positive-weight sources
represent the aligned label. Missing-label censors and untouched exclusions
stay unchanged. Spatial density uses the original geometric fusion. Gates
are zero without current opportunity and are cleared after fusion. The next
local update recomputes delta and both gates; distinct current source IDs are
required. Correlated detectors and unmodeled occlusion remain limitations.
This is not an exact joint RFS likelihood-product claim.

Two new fixed ablations accompany the primary: `marked_asymmetric_no_history`
sets beta=b, and `marked_asymmetric_no_mark` uses a for g_positive. Negative
support is otherwise unchanged. Previous M-SI is the exact no-negative
component control; previous mark-signed SI instead uses g_positive for both
signs. Reuse their complete nine-sequence trajectories as development controls.
If the new primary passes its gate, run both as declared controls on all 25
transfer sequences within this new experiment; they are never new primaries.

The primary and two new ablations send delta and two gates, 232 B/Bernoulli
plus a 32 B header. Old SI controls keep their native 224 B packets and exact
old fusion code. Log local pD and both gates, and pre-alignment source labels,
to independently recompute and trace all received evidence values.

## Execution and decision

Before tracking, check positive/negative/mixed scalar endpoints, conservative
zero-gate behavior, stale resets, source exclusion, unchanged spatial density
and actual packet round trips. Preflight all three new arms and exact M-SI on
development 0000 in both radio conditions. Require exact M-SI trajectory,
metrics, packet counts and original diagnostics except runtime. Register all
source before preflight and the stage after successful preflight.

Complete all nine development sequences and both radio conditions: 54 new
files, three arms. Independently rescore all five original shared marked
controls plus the two previous SI controls. Continue only if M-AE has lower
sequence-macro OSPA than M-No-age and M-ER in both conditions, and no higher
OSPA than M-CR, M-ECR-S and the no-negative M-SI control in both. This gate is
selected before preflight. Do not promote a favorable ablation on this cohort.

If it passes, complete all 25 seen-transfer sequences in both conditions with
the three new arms and both SI controls: 250 new files. These SI runs test
components of the new primary and do not override the earlier negative
candidate-expansion decision. Retain all five original common controls.

OSPA uses position only, p=2, cutoff 12 m. Report sequence-macro means and all
sequence values, 10000-sequence paired descriptive bootstrap intervals,
GOSPA missed/false squared costs, common-truth localization and native bytes.
Require direct successful MATLAB exits, completion markers and complete files.
Only satisfactory full results can inform a paper revision. Existing source,
inputs, earlier failures and the manuscript are unchanged by this experiment.
