# Coherent Gaussian Bernoulli current-evidence correction

The full selective and asymmetric experiments are available before this
round. Their small scalar gains did not satisfy the complete continuation
criteria. This round changes a structural approximation: preceding rules
add local existence log-odds increments but retain geometric spatial fusion
and its old spatial normalizer. Here the same admitted current posterior/prior
ratios modify both the spatial density and its existence normalization.

Separating likelihood from prior is established, not a new principle. See
[Wu et al., Bayesian data fusion with shared priors, Eq. 4 and Sec. III](https://arxiv.org/html/2212.07311v2)
and [Hlinka et al., likelihood consensus](https://arxiv.org/abs/1108.6214).
The proposed check is a guarded Gaussian Bernoulli implementation on the
present approximate LMB updates and heterogeneous sensing histories; it does
not assume that real detector errors or local priors satisfy exact centralized
Bayes conditions. Geometric posterior pooling itself does not multiply a
common prior M times; current likelihood underweighting is a different issue.

## Fixed rule

Retain the previous marked local tracker, positive/negative branch gates,
eligible existence weights beta, and ordinary eligible spatial weights alpha.
For each current retained local track, moment-match its updated spatial
density exactly as the existing reduction already does. Before and after
this local update, let spatial Gaussians be p_minus and p_plus. Encode their
log ratio as 15 doubles: the symmetric precision difference (10), information
vector difference (4), and log-density constant difference (1).

For a joint represented eligible label, let

    kappa_j = (1-beta_j) g_positive_j, if delta_j >= 0,
              (1-beta_j) g_negative_j, otherwise.

Current opportunity resets and missing-label rules are unchanged. The primary
`marked_gaussian_evidence` (M-GE) admits a source's ratio only if its precision
difference is positive semidefinite to numerical tolerance. Reject that
source's entire residual kappa when its minimum eigenvalue is below

    -1e-10 * max(1, norm(J_plus,2), norm(J_minus,2)).

This numerical tolerance is selected before preflight and is not tuned.
No eigenvalues are clipped. Ambiguous associations can widen a moment-matched
posterior, so a ratio need not be a log-concave information increment.

With admitted kappa, form the unnormalized spatial density

    h(x) = product_j p_plus_j(x)^alpha_j
           * product_j (p_plus_j(x)/p_minus_j(x))^kappa_j.

The precision, information vector and log constant are sums of the base
Gaussian terms and admitted log-ratio terms. Compute its integral I and
normalized Gaussian p_new. Use

    logit(r_new) = sum_j beta_j logit(r_plus_j)
                   + sum_j kappa_j delta_j + log(I).

This is the previous Bernoulli base density multiplied by the same admitted
Bernoulli posterior/prior ratios for both empty and singleton hypotheses.
With common priors, alpha=beta, unit branch gates and conditionally independent
Gaussian observations, the scalar and spatial results recover the centralized
single-Bernoulli fixture. That fixture does not prove exactness of approximate
LMB associations or heterogeneous priors in the real replay.

If the combined precision fails Cholesky, is nonfinite, or has reciprocal
condition below 1e-12, discard all residual corrections for that label and
use the unchanged conservative base. Also preserve the exact base spatial
implementation whenever no residual is admitted. Record all source rejections
and aggregate fallbacks; no silent covariance projection or track clipping.

Three fixed ablations accompany the primary: `_no_curvature` omits the
per-source precision-gain condition but keeps the necessary aggregate
integrability check; `_no_history` sets beta=b; `_no_mark` replaces positive
mark support by detection association mass. All else remains identical.
The preceding M-AE is the scalar-only spatial approximation control.

All four new arms transmit 352 B/Bernoulli plus a 32 B header: the previous
232 B packet plus 15 current spatial-ratio doubles. M-AE retains its native
232 B packet. These are common local observations and association settings,
but different transmitted summaries and byte costs; do not call this an
equal-byte or identical-message-information comparison.

## Execution and decision

Before tracking, verify centralized single-Bernoulli Gaussian fixtures with
agreeing and conflicting observations, zero-gate and one-source behavior,
curvature rejection, aggregate nonintegrability fallback, and exact actual
packet encoding. Preflight development 0000, both link conditions, all four
new methods plus native M-AE. Require exact original M-AE trajectory, metrics,
native packet accounting and original diagnostics except runtime. Record
source before preflight, then register full stages after successful preflight.

Complete all nine development sequences, both conditions, all four new arms:
72 files. Reuse and independently rescore the five original shared marked
controls plus the complete preceding M-AE as the scalar-only component control.
Continue only if the fixed primary has lower sequence-macro OSPA than M-No-age
and M-ER in both conditions, and no higher OSPA than M-CR, M-ECR-S and M-AE in
both. Retain every failed comparison and do not promote a secondary.

On passing, execute all 25 seen-transfer sequences with the four new arms and
M-AE as a declared component control: 250 files. All these real trajectories
have already informed development. No independent-test claim is permitted.
OSPA is position only, p=2, cutoff 12 m; retain all sequence results, paired
10000-sequence descriptive intervals, missed/false GOSPA squared costs,
localization on common truth, and native raw/delivered/fragmented bytes.
Require actual process exits, completion markers, source/packet checks, and
independent reconstruction of the Gaussian ratio and full fused distribution.
No preceding source, observations, outcomes or manuscript are overwritten.
