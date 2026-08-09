# V65 M24 convoy t=80 result

Convoy t=80 was registered and committed before its tracking outcome was
opened.  It was the only one of 25 cached convoy states above the unchanged
1% network-risk event budget.  V65 selected formation `[1]`, covered
`87.281%` of the observable rescue mass, incurred only `0.036%` useful loss
relative to selected rescue, and preserved all safety and communication
constraints.

| Mean tracking | Worst sensor | Minimum formation | Window consensus | Terminal consensus | Attempted bytes | Strong 5% gate |
|--:|--:|--:|--:|--:|--:|:--:|
| `+0.315%` | `+0.000%` | `+0.000%` | `+0.076%` | `+1.154%` | `+3.037%` | no |

The candidate is safe and weakly positive, but it does not provide the
required scene-generalized tracking gain.  Its effect is spatially narrow:
formation 1 improves by `1.381%`, the other three formations are unchanged,
and only sensor 2 has a material three-step E-OSPA reduction.  The mean gains
at the three return times are `+0.464%`, `+0.242%`, and `+0.235%`, so the
initial benefit does not spread through the network within the intervention
horizon.  Mean cardinality error changes only from `10.222` to `10.167`.

This is a post-outcome attribution, not a reason to retune the frozen 1%
threshold.  The counterexample identifies the missing state variable in V65:
additive existence rescue measures the amount of posterior dilution removed,
but not how many receiver decisions change or how far those changes can
propagate through the time-expanded fusion graph.  The next method must add a
current-only decision-breadth and causal-influence term before another fresh
tracking outcome is opened.

Evidence boundary: opened M24 development state; no held-out or validation
claim.
