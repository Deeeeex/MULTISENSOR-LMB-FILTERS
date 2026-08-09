# V66 influence-aware decision-breadth attribution result

V65 correctly detects how much receiver-supported existence mass is protected,
but the M24 convoy t=80 counterexample shows that amplitude alone does not
measure whether an intervention changes enough tracking decisions to matter at
network scale.  V66 therefore retains every V65 safety and information guard
and adds a current-only gate on robust threshold-changing decisions weighted by
their three-round influence through the registered KLA graph.

The gate was frozen at a network-time-normalized robust decision exposure of
`0.05` before any new relay outcome was opened.  The five already-opened
development states give the following attribution result.

| Scale / scene | Time | V65 risk | Robust exposure | Crossings | Affected receivers | V66 action | Known tracking gain |
|:--|--:|--:|--:|--:|--:|:--:|--:|
| M24 radial | 104 | `2.515%` | `0.1849` | 7 | 1 | enable | `+6.405%` |
| M24 radial | 124 | `2.099%` | `0.1836` | 7 | 2 | enable | `+8.446%` |
| X36 radial | 72 | `1.512%` | `0.0902` | 6 | 4 | enable | `+5.847%` |
| X36 radial | 100 | `2.267%` | `0.1969` | 13 | 4 | enable | `+9.329%` |
| M24 convoy | 80 | `1.030%` | `0.0193` | 2 | 1 | fallback | `+0.315%` |

All four previously known strong events remain enabled, while the weak convoy
event is rejected.  This is the first strict development separation between
the strong M24/X36 events and the scene-generalization counterexample without
reading future tracking outcomes at decision time.

The result supports a narrower causal statement: a useful event needs not only
enough protected posterior mass, but also enough robust decision changes and
enough opportunity for those changes to influence the network.  It does not
yet establish scene generalization because the formula and threshold were
designed after all five outcomes were known.  The next falsification test is a
fresh M24 linear-relay seed scanned only from current posterior and past
topology; at most one event may be frozen before its tracking outcome is
opened.

Generated evidence:
`RUN/GA/dynamic_topology/evidence/tracking_aligned_v66/influence_breadth_attribution/INFLUENCE_AWARE_DECISION_BREADTH_V66_ATTRIBUTION.md`.

Evidence boundary: opened development attribution only; no held-out,
validation, or learned-model claim.
