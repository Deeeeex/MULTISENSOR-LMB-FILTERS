# V95 matched-static finding

## Decision

V95 is rejected as a cross-scale dynamic-routing method. It is structurally
feasible at M24 and X36 and preserves the static route's per-sender and total
message counts exactly, but it does not produce a significant or stable
tracking gain. Full-episode tracking and scene expansion remain closed.

## Fair baseline

The hard baseline is the current physical-tree route held static for the whole
window. All four arms use the same cached posterior, measurements, link
uniforms, filter random state, communication budget and horizon:

1. static physical tree;
2. donor edge removed for one round without adding a new receiver;
3. sender-budget reallocation for one round, then return to static;
4. the same reallocated route held fixed for the whole window.

This design separates the value of deleting an input, adding a new receiver
and dynamically returning to the static route.

## Result

| Scale | H | Static | Donor only | Dynamic | Fixed | Gain / static | Gain / fixed | Gain / donor | Gate |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|:--:|
| M24 | 5 | 125.596200 | 125.596211 | 125.595824 | 125.699438 | +0.000299% | +0.082430% | +0.000308% | fail |
| X36 | 7 | 131.503832 | 131.473215 | 131.171915 | 130.945110 | +0.252401% | -0.173206% | +0.229172% | fail |

The frozen gate required at least 5% gain over both static and fixed controls,
positive gain over donor-only, and no regression in worst-sensor,
minimum-formation, consensus or attempted-byte metrics. Neither scale passes.

## Interpretation

M24 is effectively invariant to the 0.05 residual-token move: the dynamic
E-OSPA change is only 0.000299%, while window consensus worsens by 0.199%.
The truth-free novelty score is therefore not a useful effect-size predictor
at this anchor.

X36 has a real but small positive response: dynamic routing improves mean
E-OSPA by 0.252%, worst-sensor E-OSPA by 0.305%, consensus by 0.514% and
attempted bytes by 0.754% relative to static. However, the same reallocation
held fixed improves mean E-OSPA by 0.425% relative to static, so the one-round
return decision loses 0.173% to its fixed control. The fixed arm also regresses
the weakest sensor and formation, so it is not an acceptable replacement.

The main failure is methodological rather than a missing baseline: a local
posterior-novelty objective can identify useful recipients on X36, but a fixed
0.05 trust token and preselected one-round duration are too weak and too
myopic to yield a cross-scale effect. The data do not support calling V95 an
effective dynamic-routing method.

## Next boundary

The current operational reference remains the static physical tree. Do not
retune V95 on these opened anchors or expand it to convoy, relay, merge-split
or curved-corridor scenes. A successor should estimate multi-step tracking
value and jointly choose receiver, trust allocation and persistence under the
same exact message budget. Its first evidence must again include matched
static, donor-only and fixed-route controls before any larger experiment.

These are opened development results, not held-out validation claims.
