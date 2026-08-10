# V87 time-expanded effective KLA graph finding

## Primary source result

The frozen current-only probe passed at both primary scales.  It composed
two mixture-aware KLA rounds from the same opened posterior state and did
not read truth, future measurements, route outcomes or tracking scores.

| Scale | Anchor | Enabled paths | Coverage | Aggregate existence net | Aggregate state net |
|:--|:--|--:|--:|--:|--:|
| M24 | t=104, `2 -> 11` | 4/5 | 80% | +1.438995% | +0.012664% |
| X36 | t=112, `18 -> 23` | 5/5 | 100% | +27.756163% | +0.121703% |

M24 receiver 8 is the single rejected path: its existence net is
`-0.539884%` and its state-alignment net is `-0.003569%`.  The other four
M24 receivers retain between `25.752%` and `39.127%` of the first-hop
existence gain without a supported downward decision crossing.

All five X36 paths are positive.  Receivers 22 and 24 have existence nets
of `+12.332229%` and `+11.108043%`; composed KLA can therefore amplify a
useful first-hop change rather than merely attenuate it.  Every accepted
X36 path has positive state alignment and zero supported downward crossing.

## Method decision

V86 answered whether an informed gateway can transport information, but it
broadcast to every receiver in the formation.  V87 shows that downstream
KLA value is receiver-specific: physical reachability and formation
membership are insufficient action criteria.

The next executable arm must keep the causal `acquire -> broadcast ->
reference` sequence while changing only V87-positive downstream rows.  Its
frozen primary masks are:

- M24 t=104: broadcast from gateway 11 to receivers `[7, 9, 10, 12]` and
  leave receiver 8 on the reference row.
- X36 t=112: broadcast from gateway 23 to receivers
  `[19, 20, 21, 22, 24]`.

This is a path-selective routing decision, not a learned policy.  The paired
H=3 outcome remains unopened until the action bank and runtime checks are
frozen.  Passing the source gate authorizes that paired development test;
it does not establish a tracking or generalization claim.
