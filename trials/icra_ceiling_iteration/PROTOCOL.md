# Continuous direct-evidence ceiling for positive recency corrections

Registered 2026-09-08 before any ECR tracking outcome. This is a new bounded
round under the continuing method-iteration objective. Prior IR/CR/CGR and
JE/JE-R results remain immutable. All nine validation sequences and the six
previous fusion-transfer sequences are seen development data.

## Motivation and fixed rule

Joint evidence accumulation increased false-target cost. On the nine seen
sequences, most ER false outputs are supported by a current detection;
consecutive association is therefore insufficient evidence of objectness.
The released detector's unused score separates matched from unmatched
detections. Test a continuous limit on the positive age correction, keeping
the original measurement sets, local LMB likelihood, births, spatial pool,
FoV censoring, lineage eligibility, radio draws and output rule unchanged.

Let W_jm be the current local marginal association weight, normalized over
the missed-detection branch m=0 and all detection branches. For marks v_m
in [0,1], define c_j=sum_{m>0} W_jm v_m. Set c_j=0 without a current local
direct opportunity or when the measurement set is empty. Received metadata
never advances the receiver's direct time and is overwritten by the next
local update. This scalar describes current association-weighted support;
it is not a proven upper confidence bound on target existence.

For each fused label let r0 be original no-age existence, rER original ER,
and b,q the corresponding normalized eligible existence weights. Use

    c = max { c_j : eligible represented j, q_j>b_j+1e-12, r_j>=0.5 }
        (zero for the empty set),
    u = max(r0,c),
    rECR = min(rER,u).

Thus min(r0,rER) <= rECR <= rER, negative age corrections remain unchanged,
and c=0 recovers CR. The rule solves the same scalar ER KL-average objective
with the additional convex constraint r<=u. A one-source or equal-age pool
recovers the original result because rER=r0. Spatial fusion is unchanged.
The numerical weight tolerance and positive-existence test are inherited
from CGR; no new decision threshold is searched.

## Three registered candidates

* ECR-A: v_m=1 for every detection; c_j is association mass. On synthetic
  cases, assign this same mark to every true and clutter measurement. No
  truth-dependent mark is available to the filter.
* ECR-S: v_m is the released raw detector score. Treat it as a bounded
  confidence mark, not an objectness probability.
* ECR-C: v_m is a monotone logistic calibration of the released score.
  Fit a*logit(score)+b with a>=0, clipping only to [1e-6,1-1e-6]. Optimize
  sequence-balanced average logistic loss + 0.001*a^2, initial (a,b)=(1,0),
  L-BFGS-B. Fix this model and penalty without outcome-driven tuning.

For ECR-C on the nine seen sequences, train on eight sequences and apply
only to the excluded sequence (leave-one-sequence-out). A detection label
is obtained by per-frame, per-sensor minimum-cost assignment to the existing
cropped truth with the unchanged 12 m position cutoff. This is a supervised
2D matching target, not a claim of calibrated 3D detector probability.
Calibration labels and fitted coefficients never enter tracking except via
the scalar mark; the runner itself receives no truth. Report calibration
loss/Brier score using the excluded sequences, including raw-score values.
These development outcomes are not an independent method-selection test.

## Implementation and evidence gates

Copy the existing local update to a trial-local function and expose W as a
third output, without changing calculations. Verify exact posterior and
diagnostic parity on nonempty and empty updates. Check probability bounds,
miss-branch handling, zero-opportunity reset, packet round trip and all
fusion identities. Add exactly one float64 per Bernoulli (27 scalars,
216 bytes/object plus the existing 32-byte packet header); do not charge
this extra diagnostic field to historical ER/no-age baseline results.

Freeze method sources, calibration inputs/coefficients and development
protocol before tracking. Run all nine sequences under both existing radio
conditions for all three candidates and an instrumented original ER parity
arm. Independently recompute all output metrics, analytic fused existence,
same-input output counterfactuals and communication accounting; compare
against all original no-age/ER/CR/CGR/MIL-AM/TC results. Preserve adverse
results and evaluate missed/false-target cost separately.

If a candidate supports continuation, test the fixed association-only form
on all 60 cached synthetic cases and require a fair mark-only comparison
before attributing improvements to the interaction with age. Freeze a
final candidate and full-development calibration before accessing tracking
outcomes for the remaining 25 train sequences (all IDs not previously
selected). Those sequences are unused for fusion-method selection but the
released detector was trained on that split; routes may be related. Never
describe them as an independent detector benchmark test. A future candidate
change requires a separate amendment and cannot reuse outcomes as unseen.
