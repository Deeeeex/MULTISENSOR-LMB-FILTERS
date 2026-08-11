# V104 finding: receiver filtering does not explain the local regressions

## Matched result

| Arm | Mean E-OSPA | Gain | Min. formation | F6 peer terminal | Bytes saving |
|:--|--:|--:|--:|--:|--:|
| Static H=8 | 84.037151 | -- | -- | -- | -- |
| V103 broad handoff | 79.554740 | +5.334% | -0.945% | -2.948% | +5.981% |
| V104 receiver oracle | 79.555155 | +5.333% | -0.931% | -2.945% | +6.091% |

V104 keeps only nine V103 receiver rows selected retrospectively from positive
same-page tracking outcomes.  It reuses the exact frozen static result after
matching the cache SHA-256 and executes only the candidate.  Every rolling B3
window passes, runtime is 243.15 seconds, and the saved screen contains all
eight candidate pre-fusion posterior snapshots.

## What the near identity means

The receiver selection changes V103 mean E-OSPA by only 0.0005%.  F1 still
becomes the weakest formation even though none of its four V103 handoff rows
is retained.  F6 non-gateway performance is also unchanged after removing the
strongly negative receiver-33 row.  Therefore same-page receiver signs do not
identify the downstream source of harm.

Two mechanisms remain confounded in V103/V104:

1. the V101 control-only protection is extended through t=79 for every
   formation;
2. selected matured gateway rows alter information that can propagate through
   later reference and cross-formation paths.

The observed negative F1 and F6 states may be produced before, outside, or
after their own handoff rows.  V104 failure does not yet prove that effects of
different labels coexist inside a receiver, so opening a label combinatorial
oracle now would skip the simpler causal attribution.

## Next decision

V105 must run the identical H=8 protection schedule with a completely static
topology route and reuse the same frozen static outcome.  It is the only new
arm required.

- If protection-only retains the 5% mean gain and removes F1/F6 regressions,
  gateway handoff is the remaining harmful mechanism and label/receiver value
  control is warranted.
- If protection-only reproduces the regressions, the method must redesign the
  activation-age/deactivation rule before any handoff or GNN work.
- If protection-only loses the mean gain, the current 5% headroom depends on
  the coupled protection--handoff pipeline and V105/V104 differences must
  guide a joint temporal controller.

No receiver GNN or label oracle is authorized before this attribution closes.

## V105 resolution

V105 keeps every topology row static and removes all handoffs, yet obtains
5.259% mean gain with F1 at -0.931% and F6 non-gateway terminal gain at
-2.940%.  This near reproduction closes the attribution: prolonged H=8
control-only protection, not handoff receiver selection, is sufficient for
both the network headroom and the local harm.  The next experiment must vary
protection lifetime and deactivation while keeping the route static.
