# V84 braided-handover source finding

## Result

The frozen current-only scan passes its cross-scale source gate.  The same
receiver-first, mixture-aware KLA counterfactual and the same thresholds were
applied to 26 states at each scale (`seed 41`, `t = 40:4:140`).  M24 contains
three actionable sender substitutions and X36 contains eight.  Neither scale
contains a selective-protection action that passes the complete safety gate.

| Scale | Actionable states | Primary anchor | Primary edge | Local net | Sender novelty | Secondary anchor | Secondary local net |
|:--|--:|--:|:--|--:|--:|--:|--:|
| M24 | 3 / 26 | 104 | `2(f1) -> 11(f2)`, replacing `13(f3)` | 1.192% | 32.507% | 56 | 1.100% |
| X36 | 8 / 26 | 112 | `18(f3) -> 23(f4)`, replacing `25(f5)` | 9.388% | 50.681% | 88 | 4.064% |

The anchors are selected by the predeclared descending score rule with a
minimum 12-step separation.  The primary anchors are the highest-scoring
state at each scale; the secondary anchors are frozen now as stability cases,
not selected after tracking outcomes are opened.

## Interpretation

The X36 failure in the first half of the scan was not a scale or geometry
failure.  Its first valid event occurs at `t=88`, and later events appear at
`t=112,120,124,128,132,136,140`.  They include both endpoint and interior
handover directions.  Adding formations therefore extends the propagation
chain without removing the local information-owner transition that the method
needs.

This finding also rejects a threshold-tuning explanation.  No threshold was
changed after the early X36 fallbacks.  The strongest X36 event has much more
source-side headroom than the strongest M24 event, while the M24 actions remain
close to the 1% entry threshold.  The next experiment must therefore test
whether a single `0.05` sender substitution can propagate enough benefit over
three filtering rounds; source-side KLA headroom is not a tracking result.

## Frozen next gate

The first paired H=3 screen will execute the primary sender substitution only
at the current round, followed by two current-physical reference rounds.  The
candidate preserves the reference directed-message count, the replaced row's
weight multiset, current physicality, and rolling-B3 connectivity.  M24 t=104
and X36 t=112 must each improve mean tracking by at least 5%, with nonnegative
worst-sensor, minimum-formation, window-consensus, and terminal-consensus
gains.  The secondary anchors remain unopened until this primary action-family
test is complete.

If the single-edge H=3 gate fails, V85 will not lower the source thresholds.
It will instead align the decision unit with the executed action: safe
row-separable substitutions will be composed and scored as a fixed-budget
receiver-formation handover bundle, with at most one replacement per receiver
row and the same deterministic physical and rolling-connectivity projection.

This is opened development evidence.  No tracking outcome, learned selector,
validation result, or generalization claim is authorized by V84.
