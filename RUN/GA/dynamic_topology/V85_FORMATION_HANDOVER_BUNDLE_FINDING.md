# V85 formation-level handover bundle finding

## Source result

The V85 primary source gate fails at both scales.  Lowering only the
edge-novelty prefilter from `0.5%` to `0.01%` exposes additional weak safe
rows, but the unchanged aggregate `1%` net, `0.5%` novelty, and two-receiver
requirements do not produce an executable formation bundle.

| Scale | Receiver formation | Positive safe rows | Receiver coverage | Aggregate local net | Aggregate novelty | Bundle gate |
|:--|--:|--:|--:|--:|--:|:--:|
| M24 t=104 | 2 | 1 | 16.7% | 1.192% | 32.507% | fail breadth |
| M24 t=104 | 3 | 1 | 16.7% | 0.0011% | 1.629% | fail breadth/net |
| X36 t=112 | 3 | 2 | 33.3% | 0.146% | 3.184% | fail net |
| X36 t=112 | 4 | 1 | 16.7% | 9.388% | 50.681% | fail breadth |
| X36 t=112 | 5 | 1 | 16.7% | 0.0626% | 0.188% | fail breadth/net/novelty |

Both reference fallbacks retain exact message parity and rolling-B3.  No
tracking outcome is opened for V85.

## Interpretation

The V84 failure is not repaired by moving the gate from individual edges to a
same-formation sum.  The strong handover information is intrinsically
concentrated in one gateway receiver.  Existing residual-slot substitution
can expose more rows, but their KLA net effect is too small to carry the
handover signal across a formation.

The next action must therefore address temporal propagation.  A principled
three-round route has two distinct phases:

1. acquire the novel cross-formation posterior at the safe V84 gateway using
   the frozen `0.05` residual replacement;
2. on the following round, re-root the receiver formation's existing `0.70`
   dominant within-formation inputs at that informed gateway, preserving every
   row's weight multiset and message count, then return to the reference.

This acquire--broadcast schedule changes no payload and does not increase
trust.  It uses the high-weight edge only after the gateway has incorporated
the novel posterior, directly targeting the time-expanded dilution observed
in V84.  The same structural rule applies to four- and six-formation scenes.

V85 closes same-round residual-slot aggregation.  Model training and secondary
anchors remain closed.
