# V33 safe alternative-gateway trust backoff

## Question left by v32

V32 changes only one formation gateway cut at t=74 while preserving the
formation-level cycle, dominant route, residual weight 0.05, and message
count. All 12 reference-anchored alternatives fail the label-retention gate.
The three routes with useful one-round disagreement headroom improve that
metric by 0.547%--0.646%, but each also causes six supported labels to cross
below the 0.5 decision threshold. Fixed reference-strength trust is therefore
too aggressive even when the sender and receiver are changed locally.

This does not yet show that alternative sources are useless. A low-weight
conflicting input can still carry enough common information to reconnect the
network without receiving the full reference trust. V33 asks whether the
alternative gateway can be retained after its influence is projected down to
a label-safe level.

## Frozen two-stage action construction

The first stage reuses the frozen v32 candidate bank; it does not search a
new graph family. A structural route enters v33 only if its exact v32
weight-0.05 disagreement improvement is at least the preregistered 0.25%
margin. This source-only screen retains candidates 8, 10, and 12 and excludes
routes whose maximum available headroom is already too small.

The second stage changes only the two alternative cross inputs introduced by
that route. Their residual weights are evaluated on the frozen grid 0.0125,
0.025, and 0.0375. Every unchanged residual edge remains at 0.05; the dominant
route remains at 0.70; removed trust is returned to the corresponding
receiver self weight. The selected topology, message count, payload type, and
formation-level directed cycle therefore remain constant across the weight
grid.

For each structural route, the largest weight passing the exact label safety
projection is recorded. A route is executable only if that safe weight still
reduces exact one-round expected posterior disagreement by at least 0.25%.
Among executable route--weight pairs, the controller chooses the largest
disagreement improvement, breaking ties in favor of greater retained trust
and then the lower frozen v32 candidate index. Reference is the mandatory
fallback.

## Safety and scalability

The safety gate is unchanged from v31--v32: retention risk at most 0.01, no
formation mean expected-cardinality loss below -0.05, at least 80% retention
for every reference-supported label, no downward 0.5-threshold crossing, and
rolling-B3 connectivity at both sensor and formation levels. Physical
reachability, row-stochastic weights, message-count parity, dominant-route
parity, and formation-graph parity are hard invariants. No target truth,
future measurement, or later tracking outcome is used.

V33 evaluates only P times L complete routes, where P is the number of v32
routes with sufficient source-only headroom and L is the fixed trust-grid
size. For the current M24 state, P=3 and L=3, so the new preflight has nine
exact route evaluations rather than an unconstrained joint graph--weight
search. At larger scales, the same screen can cap P before the exact safety
projector; a learned model could later rank P, but it cannot replace the
projector.

## Evidence boundary

The first run may use only the frozen v30 t=74 causal state and official v32
preflight. It must stop before tracking unless a clean committed preflight
finds a safe nonreference route above the 0.25% disagreement margin. Passing
would authorize only one paired rerun of the already-opened t=72--t=74 M24
outcome. Failure closes this trust-backoff mechanism without opening another
M24 state, GNN training, X36, X48, or reserved validation evidence.
