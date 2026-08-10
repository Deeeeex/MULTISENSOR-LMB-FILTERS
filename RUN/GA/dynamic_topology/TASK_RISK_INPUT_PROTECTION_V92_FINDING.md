# V92 single-formation input protection: matched-baseline finding

## Decision

Reject V92 and do not run a full tracking episode. The action is often safe,
but its improvement over the matched V91 static physical-tree baseline is too
small and does not survive the frozen cross-scale headroom gate.

## Result

| Scale | Actionable snapshots | Median positive task-risk gain | Peak safe gain | Minimum worst-receiver gain | Minimum worst-formation gain | Gate |
|:--|--:|--:|--:|--:|--:|:--:|
| M24 | 26/26 (100%) | +0.140% | +1.288% | +0.000% | +0.000% | fail |
| X36 | 26/26 (100%) | +0.133% | +0.358% | +0.000% | +0.000% | fail |

The frozen gate required at least 10% actionable snapshots, at least +0.25%
median positive gain, and at least +1.0% peak safe gain at both scales. M24
failed the median requirement; X36 failed both median and peak requirements.

## Interpretation

Suppressing one low-weight cross-formation residual input bundle can protect a
small receiver tail without regressing any monitored receiver or formation,
but one formation at a time has insufficient network-wide amplitude. This is
evidence for testing joint formation protection as an exact action-family
upper bound, not evidence for lowering the gate or starting full E-OSPA runs.

The reported percentages are one-round truth-free posterior task-risk changes,
not E-OSPA changes or recursive tracking guarantees. Every candidate used the
same cached posterior state, physical graph, link probabilities, receiver
semantics and V91 static-route reference.
