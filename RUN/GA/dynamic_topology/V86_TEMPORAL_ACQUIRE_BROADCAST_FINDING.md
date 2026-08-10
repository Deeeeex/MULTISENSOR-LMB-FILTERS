# V86 temporal acquire--broadcast finding

## Paired H=3 result

The frozen sequence was `acquire, broadcast, reference`.  Acquire reused the
V84 `0.05` mixture-aware handover edge.  Broadcast promoted that gateway into
the existing `0.70` input slot of the other five receivers in its formation
while retaining each displaced dominant sender at `0.05`.

| Scale | Mean tracking | Receiver-formation gain | Window consensus | Terminal consensus | Attempted bytes | B3 |
|:--|--:|--:|--:|--:|--:|:--:|
| M24 t=104 | +0.060% | F2 +0.212% | -0.851% | -0.174% | +0.314% saving | pass |
| X36 t=112 | +0.298% | F4 +1.635% | -0.213% | -0.518% | +0.558% saving | pass |

Both scales preserve the exact directed-message count and every receiver
row's positive-weight multiset.  Neither scale passes the frozen 5% strong
gate, and both worsen consensus.

## What the decomposition changes

V86 is not merely another zero-signal action.  In X36, the six receivers in
F4 improve by approximately `0.75%--3.00%`; the formation mean improves
`1.635%`.  The network mean is only `0.298%` because the other five formations
are essentially unchanged.  The two-round route therefore transports some
of the strong V84 gateway information inside its formation, but one local
handover cannot create a five-percent 36-node network improvement.

M24 is different.  Its V84 source margin was only `1.192%`; V86 leaves most
broadcast receivers nearly unchanged and improves F2 by only `0.212%`.
Increasing downstream breadth cannot manufacture information that the
gateway did not acquire strongly.

The result separates two problems that previous single-number gates mixed:

1. **transportability:** whether a gateway's acquired label information
   survives a second KLA operation and changes downstream receiver estimates;
2. **coverage:** whether enough independent formations and times carry such
   transportable information to affect the full-network objective.

The full-formation V86 schedule is closed as a standalone method.  Its local
X36 gain remains useful mechanism evidence, but it is not a cross-scale or
global tracking result.

## Next method direction

V87 should construct a current-only, label-wise two-hop KLA counterfactual.
For each safe source--gateway edge, it first computes the acquired gateway
posterior, then evaluates that posterior as a candidate sender for every
physically reachable downstream receiver.  A downstream route is useful only
when the source's novel label support survives both KLA operations, changes
the receiver's existence or state estimate in the supported direction, and
outweighs the information lost by demoting the incumbent sender.

This produces a **time-expanded effective KLA graph** whose path value is
defined by posterior composition rather than graph reachability.  The source
stage must scan multiple times and scene styles at M24 and X36, select only
positive downstream receivers, combine simultaneous gateways under the same
message and rolling-B3 budgets, and measure how many formations are covered.
H=3 remains a local mechanism diagnostic; the original 5% target is retained
for the resulting repeated full-episode policy.  GNN training stays closed
until this action space demonstrates positive cross-scale headroom.
