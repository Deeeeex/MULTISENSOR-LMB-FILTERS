# V243 dynamic-topology scene selection finding

## Decision

Use `formation-braid` as the primary causal topology-repair scene at M24 and
X36.  Use `crossing` only as a secondary gateway-versus-tree repair
diagnostic.  Keep convoy, relay, merge-split, target-overlap,
curved-corridor, and braided-handover as cross-style tracking controls rather
than evidence that dynamic routing is necessary.

## Geometry-only evidence

The V243 survey evaluates physical reachability and scene validity without
running the tracker or reading target-tracking outcomes.

| Scale and style | Physical formation graph | Initial tree feasible | Exact registered route physical | First exact-route failure | First tree failure | Role |
|:--|--:|--:|--:|--:|--:|:--|
| M24 formation-braid | 100.0% | 43.1% | 33.1% | 54 | 70 | primary tree-repair stress |
| X36 formation-braid | 100.0% | 33.8% | 30.0% | 49 | 55 | primary tree-repair stress |
| M24 crossing | 100.0% | 100.0% | 95.0% | 153 | -- | late gateway-only diagnostic |
| X36 crossing | 100.0% | 98.1% | 86.3% | 139 | 158 | late gateway then tree diagnostic |
| Convoy and relay, both scales | 100.0% | 100.0% | 100.0% | -- | -- | frozen non-radial controls |
| Other surveyed styles, both scales | 100.0% | 100.0% | 100.0% | -- | -- | additional cross-style controls |

All sixteen surveyed scene-scale pairs pass the current scene validator.  The
formation graph remains connected at every step in every pair, so the
candidate scenes do not require store-and-forward semantics.

## Experimental matrix

1. Establish mechanism value on M24 formation-braid with a matched fixed-tree
   dropout arm and a causal repair arm.
2. If positive, add the minimum formation backbone as the communication arm
   and reuse the same recorded references.
3. Repeat the frozen method on X36 formation-braid without retuning.
4. Use crossing to separate gateway reassignment from formation-tree
   replacement.  Because the current failures occur near the episode end,
   do not use it as the primary significance case.
5. Use at least convoy and relay as held-out non-radial controls.  In these
   scenes the method should reduce communication without inventing topology
   changes, while preserving tracking and consistency relative to their
   static routes.

## Method implication

Dynamic routing has two distinct operations:

- **gateway reassignment:** keep the same formation tree but replace a
  sensor-level gateway that is no longer physical;
- **tree repair:** replace one or more formation edges only when the incumbent
  tree is no longer realizable.

The causal controller should report these operations separately.  A learned
edge-value model, if added later, may rank feasible gateways and replacement
edges, but it must not decide whether a physically infeasible edge remains in
the executed graph.

## Evidence boundary

This finding selects scene roles from geometry and communication
reachability.  It does not establish an E-OSPA, RMSE, consistency, byte, or
generalization improvement.  Those claims require the paired full-episode
tracking matrix above.
