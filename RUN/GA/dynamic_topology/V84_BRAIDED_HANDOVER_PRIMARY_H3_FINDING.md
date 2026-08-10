# V84 braided-handover primary H=3 finding

## Paired result

The two primary anchors frozen before tracking was opened both fail the
strong H=3 gate.  Each candidate changes one `0.05` cross-formation sender at
the current round and then uses the current-physical reference for two rounds.
Reference and candidate have identical predecision posteriors, measurements,
delivery draws, filter RNG, directed-message count, and per-row weight
multiset.

| Scale | Anchor | Source local net | Mean tracking | Worst sensor | Minimum formation | Window consensus | Terminal consensus | Attempted bytes | B3 |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|:--:|
| M24 | 104 | 1.192% | +0.0566% | +0.000% | +0.000% | -0.1466% | +0.0000% | -0.0628% | pass |
| X36 | 112 | 9.388% | +0.0004% | +0.000% | +0.000% | -0.4432% | -0.9679% | +0.0023% | pass |

Neither scale reaches the required 5% mean improvement, and both worsen
window consensus.  Secondary-anchor scoring and model training remain closed.

## Method conclusion

The failure is not caused by a missing information-owner transition.  The
source scan found three actionable M24 states and eight actionable X36 states,
including a 9.388% receiver-domain KLA improvement at the tested X36 anchor.
The failure occurs after that local counterfactual: changing one row of a
24- or 36-node fusion system affects too little of the three-round
time-expanded graph to change network tracking materially.

This closes the single-edge action family.  Repeating the same test at the
secondary anchors or increasing the edge weight would not address the causal
bottleneck: V83 already showed that larger trust can be harmful, while V84
shows that even a strong local KLA change is diluted when only one receiver is
changed.

## V85 direction

V85 changes the action unit, not the thresholds.  Safe substitutions at
distinct receiver rows are single-round separable, so they can be composed
into one receiver-formation handover bundle while retaining at most one
replacement per row.  The 1% net and 0.5% sender-novelty gates will be applied
to the aggregate receiver-formation action rather than independently to each
edge.  The deterministic projection will still require current physicality,
exact directed-message parity, per-row weight-multiset parity, and rolling-B3.

The next source calculation must first establish that such bundles change
multiple receivers at both M24 and X36.  Only then will a new paired H=3 run be
opened.  A learned selector remains out of scope until the bundled action
family clears the cross-scale tracking gate.

This is opened development evidence, not a validation or generalization
claim.
