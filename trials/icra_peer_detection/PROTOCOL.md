# Peer current-detection support for extra negative evidence

The current-pruning-information family is closed at commit 2366d098. It
removed all 3,559 target-window prediction-gap returns under the reliable
link, but GCE still had zero target detections in that window and lost to
equally informed No-age. It is not combined with this proposed method.

An exploratory inspection of its already exposed frame 53 found one nearby
GCE candidate: ordinary KLA existence 0.562127, actual GCE existence 0.333551.
Its original history adjustment is zero, both source curvature checks pass,
and the spatial-integral change is approximately 1.8e-13. An extra negative
log-odds contribution of -0.941967 causes the drop, while positive extra
support is nearly zero. Replacing that one scalar at the fixed input includes
the target in the MAP output. On original GCE states, the analogous single
replacement is still insufficient. These are exploratory single-step facts,
not a recursive repair or a selection result.

The already completed positive-only, no-mark, joint-positive-admission,
global scan scaling, persistent miss-history, curvature-conflict veto and
range-calibration families remain closed. The new question concerns how
another represented source's *current* detection support changes confidence
in an **additional** negative ratio. It neither enhances the old positive
gate nor removes ordinary local missed-detection evidence.

## Single primary rule

For each source j representing the aligned label, r_j is its executed local
posterior existence, a_j is its association mass conditional on existence,
and current opportunity is required. Set pi_j = r_j * a_j for a current,
represented, positive-weight source; set pi_j = 0 otherwise. This is a local
approximate joint existence-and-detection probability, not a calibrated
cross-source truth or visibility probability.

The primary rule `peer_joint` replaces the extra negative gate by

    g_negative_new_s = g_negative_old_s * (1 - pi_other).

There are exactly two sources. This is a bounded peer-support modulation,
not an independence claim or a new likelihood factor. The old positive
gate, local update, nominal pD=0.9, beta/history, matching, missing-source
eligibility, and observations stay unchanged. The modified negative
exponent applies to both scalar and Gaussian parts of that source's ratio;
recompute the original source-curvature and aggregate-integrability rules.
If extra evidence was ineligible because a source Gaussian is missing,
retain that original behavior. No target label, truth, frame, range fit,
threshold, coefficient, smoothing or history state enters the rule.

Retain three controls with no secondary promotion:

| Rule | Negative multiplier | Positive gate |
|---|---|---|
| original | 1 | original |
| peer_conditional | 1 - a_other on current eligible inputs | original |
| peer_joint (primary) | 1 - r_other * a_other on current eligible inputs | original |
| no_negative | 0 | original |

The conditional and no-negative controls identify the roles of current
association and posterior existence. They cannot replace a failed primary.
Preserve the native distribution exactly whenever the final admitted
exponents are unchanged. Record every proposed multiplier, gate, admitted
exponent, Gaussian, normalizer, output set and score.

## Complete fixed-input screen

Reuse exactly the 56 original nominal source runs from the previously frozen
joint-admission screen: all nine V2V4Real development segments and all five
already exposed V2X validation segments, both links, and separate GCE and
Guarded Scalar states. The diagnosed v2xt_0001 trajectory is excluded from
selection. All inputs are exposed development evidence. The screen does not
feed substituted outputs into the next frame and cannot establish recursive
accuracy or physical byte costs.

Verify the original distributions and sets first. Reconstruct all local
moments, transmitted ratios, positive/negative gates, source joins and raw W
where recorded; disclose older sources without raw W. Test exact-assignment
joint-probability identities, source permutation, absent/stale/excluded
sources, zero/one support, unchanged positive gates and multiplier bounds.
An independent verifier must reconstruct each density from encoded ratios,
recompute every multiplier, extract MAP sets with a separate cardinality
recurrence, and rescore every robot frame.

Advance only if `peer_joint` on the original GCE states strictly lowers
sequence-macro OSPA and does not increase GOSPA in each of the four
dataset/link groups. This is eight comparisons. Keep all four rules, both
source backends, every sequence, recording grouping and error decomposition.
If any comparison fails, close this exact family without fitting a power,
strength, horizon, threshold, sign subset, source backend or favorable group.

If the screen passes, separately freeze full native recursion on all 14
screen segments for the primary under GCE and Guarded Scalar. Include
original GCE, original Guarded Scalar, No-age and the two mechanism controls,
reusing verified controls where possible and giving each changed rule its
own recursion. Carry pi through an explicitly serialized current packet
field, with exact round trips and attempted/delivered/padded byte accounting;
do not use an uncharged lookup of the peer's local state. Before extension,
require at least 1% lower OSPA and GOSPA than original GCE in each dataset/link
group, strictly lower OSPA/GOSPA than No-age and matched primary Guarded
Scalar in every group, and wire cost at most 105% of original GCE. A passed
screen alone cannot revise the paper or support general method-value claims.

Only a separately frozen full-cohort evaluation and genuinely unused
recording can address stability and generalization after successful native
gates. Preserve all previous failed families, source hashes and results.
