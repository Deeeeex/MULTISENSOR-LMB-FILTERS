# V249 horizon-value projected minimum tree

## Method decision

The primary learned decision should be **which feasible formation tree carries
the minimum-message LMB-KLA backbone**, not whether to append an unconstrained
set of posterior messages.  For a physical formation graph at page `t`, every
candidate spanning tree `T` is expanded into the same sensor-level route:

- one directed local cycle in every formation;
- one bidirectional gateway pair for every edge of `T`; and
- exactly `N + 2(F-1)` posterior messages.

Thus tree choice changes information travel paths without changing the
architecture-level message count.  The deterministic projection guarantees
physical feasibility, instantaneous strong connectivity and the exact message
budget; the learned component owns only a value score.

## Why the current one-step proxy is insufficient

V246 evaluates each omitted residual edge with an exact current-round mixture-
aware KLA counterfactual.  It is an appropriate one-step diagnostic, but it has
two structural limitations:

1. enumerating delivery outcomes for every candidate on every page is too
   expensive to scale from M24 to X36; and
2. a useful topology change may alter a gateway posterior now and improve a
   downstream formation only after several recursive fusion pages.

The repository already contains a stronger warning.  V87 classified paths by
current posterior-existence and state-alignment changes, but V88 showed that
the proxy sign did not predict the paired tracking outcome.  A supported label
can be a false track, and lower posterior disagreement can accompany worse
cardinality or E-OSPA.  V249 therefore does not train on posterior-change signs
and does not treat an analytic disagreement score as task value.

## Finite-horizon teacher

At a captured causal state `s_t`, let `T_0` be the V242 min-edit tree and let
`T` be another physically feasible tree with a valid distinct-gateway
assignment.  The offline teacher replays `T_0` and `T` from the same state for
`H=3` pages using identical measurements and delivery uniforms, then records
the vector

`q_H(s_t,T) = [gain_E, gain_RMSE, gain_consensus, gain_tail, byte_saving]`.

Truth, future measurements and future delivery outcomes are used only to form
offline labels.  They are unavailable to the deployed policy.  A tree is a
positive teacher action only when E-OSPA, RMSE and consistency all improve,
the registered weakest-formation tolerance is respected, and attempted bytes
remain below the corrected fixed-tree reference.

Before model fitting, an oracle screen must establish non-trivial headroom:

1. enumerate all feasible M24 trees at the real V247 cut windows;
2. retain a fixed structurally diverse top-K bank at X36/X48 rather than
   enumerating a combinatorial raw set; and
3. stop this method branch if no tree beats the min-edit tree on the joint
   finite-horizon target.

This prevents training a GNN for an action space that has no useful member.

## Causal graph features and learned score

A permutation-equivariant formation GNN consumes only current observable
state:

- node summaries of label-existence distribution, spatial uncertainty,
  current detection opportunity and recent delivery history;
- edge summaries of physical reachability, reliability, persistence, gateway
  payload bytes, posterior complementarity and normalized distance; and
- the incumbent tree, edge age and normalized graph-size descriptors.

The GNN produces one globally conditioned score per feasible undirected
formation edge.  A tree score is the sum of its edge scores, and a deterministic
maximum-weight spanning-tree projection selects the executed tree.  Because
each edge score is conditioned on message-passed global context, the model is
not restricted to a purely local edge heuristic even though the final
projection is additive.

Mandatory baselines are a physical min-edit tree, a reliability/distance tree,
a linear or ridge edge scorer using the same features, and the finite-horizon
oracle.  The GNN is justified only if it improves complete held-out
scene-seed trajectories over the simpler learned baseline.

## Safety and stability projection

The learned score cannot relax the following constraints:

1. every selected formation edge is physically realisable in both directions;
2. distinct sensor gateways exist for all incident tree edges;
3. the expanded sensor graph is strongly connected;
4. every KLA row remains non-negative and sums to one;
5. the directed message count is exactly `N + 2(F-1)` and the attempted-byte
   cap is checked from the selected gateway payloads; and
6. the incumbent tree is retained unless it becomes infeasible or the
   calibrated lower value of switching exceeds a frozen hysteresis cost.

If no learned proposal passes, the policy falls back to the V242 min-edit
tree.  These are executable guarantees; predicted tracking improvement is not
one and must be established empirically.

## Structural theorem used by the paper

For `F` formations, assume each formation's local directed cycle is strongly
connected and every undirected edge of a formation spanning tree is realised
by one message in each direction.  Contracting each local cycle gives the
bidirected formation tree, which is strongly connected.  Expanding the
contraction therefore preserves strong connectivity.  The route contains `N`
local-cycle messages and `2(F-1)` gateway messages, hence exactly
`N + 2(F-1)` messages.  This theorem is independent of the learned scores.

The claim is intentionally architecture-specific; it is not a global lower
bound over arbitrary directed graphs.

## Evidence sequence

1. Finish V248 and determine whether full causal repair and the V242 minimum
   backbone have tracking value on the corrected V247 M24 scene.
2. Use V248 causal checkpoints to run the bounded M24 H=3 tree oracle at the
   actual cut windows.  Do not train if the oracle lacks joint headroom.
3. Freeze the candidate-bank construction, value vector, tail tolerances and
   horizon before collecting additional scene-seed trajectories.
4. Fit ridge and GNN scorers on grouped trajectories; calibrate switching
   lower bounds on disjoint complete trajectories.
5. Evaluate the frozen policy on unseen M24 and X36 seeds, then X48 and at
   least two non-radial styles such as relay, merge-split or curved corridor.

V249 is a method specification until the oracle and closed-loop paired results
exist.  It makes no current tracking-gain or generalization claim.

## Structural falsification on the corrected scene

The M24 V247 physical formation graph was inspected at the registered cut and
handover anchors `t=[70,84,151]`.  At every anchor it contains exactly three
undirected formation edges for four formations.  The graph is already a tree,
and the exact V249 enumeration returns `[1,1,1]` feasible spanning trees,
including the V242 reference.  There is therefore no alternative formation
tree to score at any of the registered task-coupled windows.

This falsifies the V249 action definition before any H=3 tracking rollout or
GNN training.  The result does not falsify minimum-budget routing itself.  It
shows that the useful degree of freedom lies one level lower: the same unique
formation skeleton admits many physical sensor-to-sensor gateway embeddings.
At the three anchors, the directed incoming-gateway assignment counts are
respectively `[1080,36,1080,36]`, `[1080,36,1080,36]` and
`[1050,36,36,1050]` by formation, corresponding to approximately
`1.51e9`, `1.51e9` and `1.43e9` global combinations before bounded candidate
construction.

The next method should therefore keep the V242 formation tree, local cycles,
message count and safety projection fixed while selecting a small causal bank
of sensor-level cross-formation gateway assignments.  A finite-horizon oracle
must again demonstrate joint tracking headroom before fitting a value model.
