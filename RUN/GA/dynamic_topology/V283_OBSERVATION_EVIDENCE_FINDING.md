# X36: observation paths are not the same as strong target evidence

Status: completed reference diagnosis and method decision, 2026-09-06.
No candidate policy has been evaluated and the best-method table is unchanged.

## What was held fixed

V282 traces steps 1--40 of the original 160-step X36 formation-braid scene,
seed 1301, using V242's route, directed delivery draws, measurements, fusion
approximation and MAP readout. The full scene is generated before truncation.
Runtime/driver source is `fbf17cd`; the 40-step process exited 0. All 1,440
E-OSPA cells and finite RMSE values/masks match the stored reference prefix.
The raw trace is saved locally; compact reports and CSVs are retained in
`evidence/tracking_aligned_v282/x36_prefix40_seed1301/`.

V283 reuses that trace offline. A retained label is marked as having an
observation-opportunity path if positive component-mean detection probability
occurred locally or entered through an actually used positive-weight fusion
input. Detections, missed detections and active observable-absence censoring
all count. Pruning removes the flag. This is not confirmed-detection
provenance, target-truth matching or a measurement of retained information.

## Decision-relevant results

The local implementation sets detection probability to zero outside its FoV.
Across 27,939 zero-pD label stages, maximum absolute local existence change
is 2.22e-16. This trace therefore does not support an explanation based on
applying the nominal missed-detection penalty outside the implemented FoV.
It does not establish the accuracy of the component-mean FoV approximation.

At step 40 the mean local MAP count is already 5.806 before the final fusion,
versus 5.861 at output. The local/pre-spatial/fused existence masses are
6.020/6.153/6.062. The immediate spatial term is not the only place to look;
earlier cumulative spatial losses remain possible. Pooling can add
neighbor-only labels, so a pooled-minus-local mass is not a per-target loss.

| Diagnostic | Steps 1--5 | Steps 36--40 | Interpretation |
|:--|--:|--:|:--|
| Mean weight of inputs with no observation-opportunity lineage | 0.7780 | 0.0083 | Untouched states dominate startup, but rarely remain untouched late in this prefix. |
| Their share of negative weighted log-odds magnitude | 89.860% | 0.937% | Ongoing untouched-prior dilution is not a sufficient account of the late weak pools. |
| Mean weight of inputs with zero current component-mean pD | 0.8595 | 0.7949 | Not observing now is very different from never having received an opportunity path. |
| Weak pre-spatial pools / all label pools | 3,756 / 4,320 | 3,227 / 4,264 | Weakness persists after opportunity flags have spread. |
| Weak pools without an active input r >= 0.9 | 3,667 / 3,756 | 3,117 / 3,227 | In 96.591% of late weak pools, no currently available input meets this descriptive strong-input level. |

Here a weak pool has weighted input log odds below zero, before spatial
overlap. The 0.5/0.9 levels describe the saved inputs; neither is a new
output threshold or a tuning parameter. Counts are label-fusion events,
not missing true targets. The weight columns are averaged over all label
pools, while the negative-log-odds share uses the sum of magnitudes of
negative weighted contributions as its denominator.

The late-prefix result does **not** rule out a lasting cold-start effect.
Changing prior treatment in the first few steps could change all later
states; no such intervention has been run. Nor does a historical opportunity
flag establish that informative positive evidence survived. It may trace a
missed detection, a tiny positive weight, or already weak historical belief.

## Method decision

Keep the sparse routing backbone as the communication-budget reference.
The next design problem is to preserve and move useful observation evidence
over repeated hops, distinguishing its source and time from recycled local
belief. Do not equate a connected path, an opportunity flag, and a strong
target estimate. Do not declare freshness-only copying or source preservation
a validated method; previous V268/V269 source-preservation interventions also
had set-error tradeoffs.

Current-FoV-only exclusion is not justified: it would discard much of the
historical information that lets a node track outside its own view. Always
preferring high existence is also unsuitable because negative observations
are legitimate evidence. Lowering the output threshold or adding an eta
floor does not answer the transport question. A pure untouched-prior mask
would require a causal startup comparison, not a claim of effectiveness from
this diagnostic.

A useful next candidate must explain which observation evidence remains
represented after each hop, how it ages, and how repeated circulation avoids
treating the same information as new. Start with a bounded X36 paired test,
including cardinality error, E-OSPA, conditional RMSE, consistency, local tail
and all proposed metadata bytes. Expand to M24 and independent seeds only
after an actual method gain, without replacing existing baselines or relaxing
the cross-scale objective. These requirements are a design direction, not
a frozen algorithm or a performance guarantee.

## Related-work boundary checked for the paper

| Primary work | Verified scope | Consequence for this study |
|:--|:--|:--|
| [G. Li et al., Signal Processing 166:107246, 2020](https://doi.org/10.1016/j.sigpro.2019.107246) | GM-PHD multi-view fusion with clustering and compensation outside common views; author PDF consulted. | This is not label-wise LMB. Our absent-label FoV censor is not an implementation of the complete CA-GCI method. |
| [S. Li et al., FUSION 2018:1201--1208](https://doi.org/10.23919/ICIF.2018.8455250) | LMB fusion with different FoVs in centralized and peer-to-peer settings; official conference/university abstract checked. | Multi-view LMB fusion is existing work; no claim about its exact implementation is made from the abstract. |
| [Wang et al., Signal Processing 150:75--84, 2018](https://doi.org/10.1016/j.sigpro.2018.04.010) | Centralized multi-view LMB with per-Bernoulli information-based GCI weights; publisher abstract/introduction checked. | Information-based label weighting alone is not a new contribution. A faithful algorithmic comparison still requires its full specification. |

Metadata for all three was retrieved by DOI content negotiation. No external
implementation was obtained or reproduced in this step. The remaining
novelty burden is a useful interaction of a sparse, lossy, finite-hop routing
budget with evidence preservation, not multi-view weighting by itself.
