# V79 balanced recovery operator finding

V79 rejects a full dominant-route rebalance as the recovery action.  The
balanced cycle fixes the linear structure that motivated the experiment, but
it creates a much larger nonlinear arm perturbation than the one it is meant
to recover.

| Case | Reference centered norm | Balanced bound | CBB peak factor | CBB terminal factor |
|:--|--:|--:|--:|--:|
| M24 historical | 1.634 | 0.996 | 8.419 | 8.395 |
| M24 aligned | 1.634 | 0.996 | 11.020 | 11.020 |
| X36 historical | 1.641 | 0.998 | 10.311 | 10.311 |
| X36 aligned | 1.641 | 0.998 | 9.318 | 9.318 |

The matrix calculation itself is correct: replacing the within-formation
index star by a cycle reduces effective column imbalance from about `2.787`
to `0.002` and places the selected one-round centered bound below one on both
scales.  The failure comes from optimizing only propagation of an existing
perturbation.  The candidate and reference arms use different recovery
operators, so a local linearization has the affine form

`delta_next = B delta_current + (B - R) x_reference`.

V79 controls the first term but ignores the second.  Swapping every dominant
sender at weight `0.70` makes `B - R` large, whereas the original direct-safe
pulse changes only a few residual senders at weight `0.05`.  The observed
component split matches that mechanism: M24 historical spatial-centered
energy jumps from `0.000303` after the pulse to `0.028520` after the first
balanced round, and X36 historical jumps from `0.000092` to `0.005787`.
Existence-centered energy also grows strongly.

The next recovery design must therefore be the closest contractive operator
to the reference, not the most balanced operator in isolation.  A first
principles candidate is a support-preserving relaxation

`W(gamma) = I + gamma (W_reference - I)`,

with the largest `gamma` whose reliability-adjusted centered operator meets a
frozen contraction bound.  This keeps every sender and message unchanged,
returns only the necessary fraction of received weight to self, and minimizes
the nominal matrix change within that one-parameter family.  A source-only
preflight must first determine whether any positive `gamma` actually provides
a nonexpansive interval on both M24 and X36; if not, the action needs a sparse
edge/weight correction rather than a wholesale cycle replacement.

V79 remains opened-anchor mechanism evidence.  It uses no prediction,
measurement, future link, truth, packet draw, route execution, tracking
outcome, or model training.
