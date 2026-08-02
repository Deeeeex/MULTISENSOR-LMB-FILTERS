# V34 gateway label-pruning threshold diagnostic

## Why a lower-weight diagnostic is still needed

V33 shows no safe route--weight pair at 0.0125, 0.025, or 0.0375. For the
three promising alternative-gateway routes, the bottleneck is always label
`[1,3]` at the newly selected receiver. Its reference expected existence is
about 0.203, but it is absent from the retained fused posterior whenever the
alternative message arrives. Because that message arrives with probability
about 0.94, the expected retention ratio remains near 2.6%--2.7% across the
entire V33 grid.

This is a threshold phenomenon rather than a smooth degradation. Before the
alternative-gateway family is closed, V34 tests whether the label reappears
at very small positive weights and whether any such weight can still provide
the registered 0.25% disagreement improvement.

## Frozen diagnostic grid

V34 keeps v32 candidates 8, 10, and 12 and changes only their two alternative
cross inputs. The low-weight grid is

`[0.0001, 0.00025, 0.0005, 0.001, 0.0025, 0.005, 0.010]`.

All unchanged residual edges remain at 0.05, the dominant route remains at
0.70, and removed trust returns to receiver self weight. The physical graph,
message count, payload type, formation-level cycle, and rolling-B3 recovery
sequence are identical to v33. The diagnostic evaluates 3 x 7 = 21 exact
one-round routes and reads no target truth or future outcome.

For each route, V34 records the largest tested weight that passes the frozen
label-safety projection and the smallest tested weight that reaches 0.25%
disagreement improvement. V33 already places an observed no-benefit point at
0.025 and an observed useful point at 0.0375. If the low-weight safe region
ends below those useful weights, the experiment establishes an empirical gap
between label survival and consensus utility for this action family.

## Evidence boundary

The diagnostic reuses only the official v30 causal t=74 state and frozen
v32/v33 source-only evidence. A clean preflight may authorize one paired
tracking rerun only if a low-weight route is simultaneously label-safe and
above the 0.25% disagreement margin. Otherwise it closes the registered
late-reconnect alternative-gateway mechanism without opening another M24
state, GNN training, X36, X48, or reserved validation evidence. It does not
claim a continuous proof for every real-valued weight; its conclusion is
limited to the frozen grid plus the already-observed v33 points.
