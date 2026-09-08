# Direct-evidence ceiling for Bernoulli existence recency

This describes the already registered method; it introduces no new fitted
parameter or rule after the reserved-cohort run begins. The fixed main arm
uses the common marked local update and raw-score support (M-ECR-S).

## Fixed-input quantities

After the existing label alignment and participation rules, let P be the
eligible represented spatial inputs and E the eligible existence inputs.
The latter may also include a qualified absence censor. Both sets exclude
zero communication weights. Untouched-prior and FoV-absence rules are those
of the original ER control. Let a be the normalized spatial weights, b the
normalized ordinary existence weights and q the normalized recency weights.

For represented sources with a previous direct opportunity at tau_j,
f_j = 0.25 + 0.75 exp[-(t-tau_j) dt / 5 s]. A represented source without
direct history uses f_j=0.25. Qualified missing-label evidence uses the
neutral f_j=1. Set q_j = b_j f_j / sum_k b_k f_k. Incoming messages do not
refresh the receiver's local timestamp. A current missed detection does
count as a direct sensing opportunity; absence of opportunity does not.

For a fixed label, let eta_a = integral product_j p_j(x)^a_j dx, with the
same finite-mixture approximation and admissibility fallback as ER. Then

    p_a(x) = product_j p_j(x)^a_j / eta_a
    L0 = sum_j b_j logit(r_j) + log(eta_a)
    LER = sum_j q_j logit(r_j) + log(eta_a)
    r0 = sigmoid(L0), rER = sigmoid(LER).

Logits use the registered numerical clamp [1e-9,1-1e-9]. The input-weight
restriction is applied before these sums. Missing-label censors have no
spatial density; omitting them from P does not omit them from E.

## Current support and the cap

For source j, normalize its current LMB association row over the missed-
detection branch m=0 and the retained detection branches m>0. Denote those
normalized weights by W_jm. For each detection, v_m is a bounded support
mark: 1 (A), its released detector score (S), or its development-fitted
calibrated score (C). Set

    c_j = sum_{m>0} W_jm v_m.

Set c_j=0 if there is no current local direct opportunity, no measurements,
or no nonzero valid association row. Thus 0<=c_j<=sum_{m>0}W_jm<=1.
The scalar is computed locally before message formation, transmitted with
the source timestamp, and reset after fusion. A remote reception cannot
create local direct evidence. Every synthetic measurement, including all
clutter, receives the same mark 1 when scores are unavailable.

Only represented positive-existence sources that gain weight may authorize
an upward recency change. With the fixed numerical weight tolerance,

    U = {j in E: represented, b_j>0, q_j>b_j+1e-12, r_j>=0.5,
                    source j has a current local direct opportunity}
    c = max({c_j: j in U} union {0})
    u = max(r0,c)
    rECR = min(rER,u).

The maximum avoids diluting the support of one newly observing source by
the number of currently uninformative sources. It also makes the rule
vulnerable to an overconfident eligible source. This is a design choice,
not a proof of robustness against arbitrary sensor errors.

## Variational characterization and exact limits

Hold eligibility, a, b, q, the input densities, and u fixed. The existing
two-block ER objective is

    J(r,p) = sum_j q_j D_B(r || r_j) + r sum_j a_j D(p || p_j).

Add only the scalar constraint 0<r<=u. Since
sum_j a_j D(p||p_j) = D(p||p_a)-log eta_a, its minimum over p for each
r>0 is attained at p_a. The remaining scalar function, up to a constant,
is F(r)=r log r+(1-r)log(1-r)-r LER. Its derivative is
logit(r)-LER and its second derivative is 1/[r(1-r)]>0. The unconstrained
minimizer is rER, so the constrained minimizer is min(rER,u). Hence:

- min(r0,rER) <= rECR <= rER.
- Every negative recency change is preserved: rER<=r0 implies rECR=rER.
- With c=0, the result is the conservative control min(r0,rER).
- Equal effective ages or one effective source give q=b and rECR=r0=rER.
- For fixed r0 and rER, raising c cannot reduce rECR; its slope is either
  zero or one away from the two clipping breakpoints.
- For fixed inputs, p_a is unchanged by either the recency weights or cap.
  Recursive associations, pruning, and state extraction may still change
  subsequent spatial estimates. Their empirical comparison therefore uses
  common truth-assignment support rather than separate surviving tracks.

Neither u nor c is an established upper confidence bound for true target
existence. The result minimizes an input-dependent constrained surrogate,
not ground-truth tracking risk. It does not guarantee better OSPA, fewer
misses, cardinality consistency, discovery preservation or consensus.
It cannot suppress a false track solely because its support is below r0:
the rule limits additional positive recency influence, rather than replacing
the complete posterior by a one-frame detector score.

## Shared marked measurement control

All marked arms use the same empirical local observation model. From the
nine development sequences, fit the registered monotone logistic map
p(s)=sigmoid(a logit(s)+b). With the corresponding sequence-balanced
positive-class proportion pi, the Bayes-odds identity gives

    ell(s) = odds(p(s))/odds(pi).

If location and mark are conditionally independent given target versus
clutter class, then g(z,s|x)/kappa(z,s) factors into the original spatial
likelihood ratio times ell(s). Multiply each detection association column
once by ell(s), leaving the missed branch unchanged. A mark common across
Gaussian components cancels in their within-measurement normalization.

The empirical two-dimensional association labels, regularized calibration,
fixed detection probability and conditional independence are approximations.
The held-out fusion cohort never fits them. Calibration estimates a local
measurement model; the later evidence cap is a decision constraint using
the same observations. It is not a second independent likelihood factor.
Original No-age/ER and the score-using No-age/ER controls must both remain
visible so observation-model improvement is not attributed to the cap.
