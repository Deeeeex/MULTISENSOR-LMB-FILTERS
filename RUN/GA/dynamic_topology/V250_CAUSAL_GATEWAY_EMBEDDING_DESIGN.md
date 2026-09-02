# V250 causal sensor-gateway embedding

## Why the action moved below the formation graph

V249 established that the corrected M24 task-coupled scene has exactly one
feasible formation spanning tree at each registered anchor `t=[70,84,151]`.
The physical formation graph contains `F-1` edges, so a formation-tree scorer
has no alternative action.  This stops the tree-selection H=3 oracle and GNN
before any tracking compute is spent.

The same formation skeleton nevertheless has a large sensor-level action
space.  For each directed formation-tree edge, one physically reachable
sender/receiver sensor pair can carry the cross-formation posterior.  Incoming
edges to the same formation must use distinct receivers so that each receiver
retains at most one cross residual slot.  The raw global assignment counts at
the three M24 anchors are approximately `1.51e9`, `1.51e9`, and `1.43e9`.

## Frozen action semantics

V250 keeps all V242 architecture decisions fixed:

- the current causal formation tree;
- one reliability-aware local directed cycle per formation;
- one message in each direction for every formation-tree edge;
- the V242 dominant/cross-residual KLA weights; and
- exactly `N+2(F-1)` attempted messages per page.

The only action is the sensor-level embedding of the `2(F-1)` directed
cross-formation messages.  Replacing a gateway returns the old residual weight
to the old receiver self posterior and moves the same residual weight from the
new receiver self posterior to the selected cross-formation sender.  The
deterministic projection checks physical reachability, distinct receivers,
strong connectivity, non-negative KLA rows, unit row sums, and the exact
message count.  If a requested gateway becomes infeasible, execution falls
back to the current V242 assignment.

## Bounded causal candidate bank

The raw assignment space is too large to enumerate with H=3 tracking.  The
first bank is deliberately small and auditable.  It contains:

1. the V242 reliability/distance reference assignment;
2. top current-physical single-arc alternatives for every directed formation
   edge, excluding receiver conflicts;
3. top joint incoming assignments for each receiving formation; and
4. a few coordinated global rank profiles assembled from the local lists.

Candidate construction uses only the current physical page, link reliability,
positions, immutable physical identities, and the causal incumbent.  It does
not use target truth, future geometry, future delivery, measurements, tracking
scores, or posterior proxy signs.  A later candidate generator may add current
posterior and detection-opportunity features, but only after the structural
bank proves that gateway changes have finite-horizon task value.

## Evidence sequence

1. Require 8--32 unique, structurally valid candidates at every registered
   M24 anchor, with at least two receiver identities represented per
   formation.
2. Capture the V242 pre-fusion state and two-page topology history once.
3. Replay every bank candidate for `H=3` with identical measurements, link
   uniforms and filter RNG.  The anchor assignment persists while it remains
   physical and otherwise falls back to V242.  The frozen oracle maximizes the
   minimum percentage gain across E-OSPA, RMSE and consistency, subject to no
   attempted-byte increase and at most 2% regression in either weakest-
   formation metric.
4. Stop if fewer than two anchors have a joint-positive candidate or if the
   aggregate oracle does not improve all three core metrics while respecting
   the frozen tail and byte tolerances.
5. Only after this gate, fit a ridge ranker on causal features.  A GNN is kept
   only if it improves complete held-out trajectories over ridge.

## Structural preflight result

The source-bound M24 preflight at `t=[70,84,151]` produced 21 unique,
executable candidates at every anchor.  The corresponding raw assignment
spaces contain `1,511,654,400`, `1,511,654,400`, and `1,428,840,000`
combinations.  The minimum receiver-identity coverage is two in every
formation at every anchor.  All candidates preserve 30 attempted messages,
physical reachability, strong connectivity and valid KLA rows.

This authorizes the paired H=3 tracking oracle.  It does not yet establish
tracking value, justify a learned ranker, or support a generalization claim.
