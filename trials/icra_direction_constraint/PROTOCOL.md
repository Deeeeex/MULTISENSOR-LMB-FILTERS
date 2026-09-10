# Limit a reversal of pooled current evidence

Start from commit 71dad7e5. The previous goal turn completed and independently
verified the peer-detection screen; its primary failed four of eight gates.
It is closed, as are the earlier curvature-pair veto, joint-positive,
miss-history, current-pruning-information and first-event interventions.
This experiment does not change their outcomes or select a secondary.

The existing complete diagnostic-label trace shows that the original GCE
and No-age existence probabilities recover to approximately 0.998 before
frame 45. Repeated extra negative updates then precede GCE's collapse.
Removing the scalar term at frame 53 cannot erase its preceding recursion.
The label and those frames motivate a hypothesis only; they never enter the
new rule or its selection cohort. The prior first-global-event intervention
is not extended, moved or repeated.

## Matched prediction reference

For each currently fused label, keep the original eligible sources, scalar
history weights beta, spatial pooling weights alpha, local posteriors and
local predictions. Form a reference using those same frozen weights on the
source predictions. Let I_minus and I_0 be the Gaussian product integrals of
the source predictions and posteriors respectively. Define

    z_ref = sum(beta * logit(r_minus)) + log(I_minus)
    z_0   = sum(beta * logit(r_plus))  + log(I_0)
    D_0   = z_0 - z_ref.

The weights are held at their current original values, including the original
history decision. This is a matched reference for this comparison, not a
new predictive Bayesian distribution or an estimated visibility probability.
No truth, label ID, frame, range, learned coefficient or temporal state
determines the reference or rule. Extra-ineligible missing-source labels
remain exactly unchanged.

Let k be the original final admitted exponents, including original source
curvature rejection and aggregate fallback. Scale the complete admitted
ratio by one common lambda in [0,1], including both its existence and
Gaussian factors:

    J(lambda) = J_0 + lambda * sum(k_j * delta_J_j)
    h(lambda) = h_0 + lambda * sum(k_j * delta_h_j)
    c(lambda) = c_0 + lambda * sum(k_j * delta_c_j)
    z(lambda) = sum(beta * logit(r_plus))
                + lambda * sum(k_j * delta_j) + log(I(lambda)).

For Guarded Scalar, keep its original spatial pool and integral throughout;
only its already admitted scalar ratio follows this same path.

## Single primary and mechanism controls

The primary `nonreversal` retains lambda=1 unless D_0 and
D_1=z(1)-z_ref have opposite signs, both with magnitude greater than 1e-7.
This fixed numerical dead band is ten times the inherited 1e-8 absolute
normalizer audit tolerance; it is not fitted to tracking scores.

For an opposite-sign pair, find the boundary z(lambda)=z_ref, and retain the
largest lambda on the connected interval starting at zero whose update has
the original D_0 direction. Use 60 bracketed bisections, keeping the safe
endpoint. Both J_0 and J(1) are positive definite, so the interpolated precision
is positive definite. The Gaussian log integral is convex in this affine
natural-parameter path. Opposite endpoint signs therefore give one crossing
inside this interval. A separate root solver verifies every crossing.

This is an empirical constraint: the original pooled direction is not proof
of correct evidence, and extra ratios can in principle correct a wrong pooled
direction. The screen explicitly tests the cost of imposing this constraint.

Retain four rules, with no alternative-primary promotion:

| Rule | Limited reversals |
|---|---|
| original | None |
| nonreversal (primary) | Both directions |
| negative_reversal | D_0 positive, D_1 negative |
| positive_reversal | D_0 negative, D_1 positive |

All rules retain original alpha, beta, prior/posterior inputs, local nominal
pD=0.9, matching and observation sets. Scaling follows the original accepted
ratios; it does not read curvature rejection as a trigger, admit a rejected
source, modify a positive/negative local gate or request another packet field.
If the final exponent vector is unchanged, preserve the native distribution
and candidate record exactly. A native implementation, if reached, must
reconstruct the reference from the actually received posterior and encoded
ratio; a hidden uncharged peer-state lookup is prohibited.

## Complete screen and fixed exit

Use exactly the preceding 56 original source runs: nine V2V4Real development
segments, five already exposed V2X validation segments, both links and the
separate original GCE/Guarded Scalar trajectories. All 14 segments are exposed
development inputs. The diagnostic v2xt_0001 is excluded from selection.
Four output rules produce 224 score rows. This is one-step substitution at
visited inputs, with no output fed back into the next frame.

Freeze code, fixtures, cohort, prior evidence, report construction and all
eight gates before alternate distributions on real inputs. Then check one
source run from each backend with both independent calculations, and complete
the full screen. The producer starts from source prediction/posterior moments.
The independent verifier reconstructs source predictions from posterior
Gaussians minus encoded increments, starts each path from the saved fused
density minus its original residual, uses Brent's method for the boundary,
and recomputes all sets and OSPA/GOSPA with separate code. Preserve every
reference integral, direction, multiplier, exponent and distribution.

Advance only if the primary on original GCE states strictly decreases
sequence-macro OSPA and does not increase GOSPA in each of the four
dataset/link groups. If any of these eight comparisons fails, close this
exact constraint without selecting a direction, changing its reference,
adding a margin/strength, fitting the numerical tolerance or choosing inputs.

If all pass, separately freeze native recursion for all 14 segments, both
links, the primary under GCE and Guarded Scalar, and both directional
mechanism controls under GCE. Reuse verified original GCE, Guarded Scalar and
No-age controls where possible. Require at least 1% lower OSPA and GOSPA than
original GCE in every dataset/link group, strict improvement over both No-age
and matched primary Guarded Scalar in every group, and wire cost no higher
than 105% of original GCE. Verify actual serialization, baseline parity,
complete native exits and all output frames before discussing promotion.

Only successful native gates permit a separately frozen full exposed-cohort
evaluation and new unused recordings. No same-input result establishes
recursive gains, communication cost, paper readiness or generalization.
