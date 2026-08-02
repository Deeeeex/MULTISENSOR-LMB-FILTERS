# V32 label-compatible alternative-gateway reconnect

## Why the reconnect mechanism must change

V30 improves the three-step M24 tracking window by keeping formations 2--4
isolated at t=72 and t=73, but rolling-B3 connectivity forces a reconnect at
t=74. Returning to the registered reference route leaves terminal consensus
1.595% worse than reference. V31 tried to repay this deficit by increasing
the weights of the same registered cross inputs. That mechanism is rejected:
the only safe pulses improve one-round disagreement by less than 0.25%, while
every stronger pulse into formations 2--4 suppresses at least one supported
label beyond the frozen retention limit.

The new hypothesis changes the information source rather than its weight.
At t=74, V32 keeps the formation-level directed cycle, high-weight dominant
route, residual weight 0.05, and directed message count exactly equal to the
registered reference. It changes only the sensor-level gateway endpoints of
the low-weight residual cycle. This isolates the proposed mechanism: any
difference comes from choosing a more compatible sender--receiver pair, not
from adding communication, strengthening trust, or changing the macroscopic
information-flow graph.

## Label-compatible K-best gateway proposal

For a physical residual edge from sender j to receiver i, the proposal score
uses only the current LMB posteriors and current link reliability. Shared
labels are compared by the Bernoulli Chernoff coefficient induced by the
candidate residual KLA weight; source-only high-quality labels provide a
small novelty term, and lower source precision incurs a penalty. No target
truth, future measurement, or later tracking outcome enters this score.

The registered residual route is a local balanced cycle inside every
formation. Choosing one cut receiver per formation and reconnecting those cut
edges along the fixed counter-clockwise formation order creates a global
sensor-level residual cycle. A completely new set of cuts can change every
cross input at once and make an average compatibility score hide a harmful
label. V32 therefore uses a radius-one topology trust region: a proposal may
change the cut receiver of at most one formation relative to the registered
route. Such a move replaces exactly the two cross edges adjacent to that
formation while leaving all other gateways unchanged.

Candidates inside this trust region are ranked by a layered K-best dynamic
program, capped at 16 complete routes. If a formation has m admissible cut
points and there are F formations, the radius-one bank contains only O(Fm)
structural alternatives; the dynamic program also supports larger frozen
radii without enumerating all m^F gateway combinations. The same action
construction therefore remains usable when the number of formations or
sensors per formation grows.

The compatibility score is a proposal heuristic, not a safety certificate.
Moment summaries can rank endpoints cheaply, but they can miss a harmful
label or a multi-source interaction. Every proposed complete route is
therefore executed through the same one-round LMB-KLA and link-delivery model
as the runtime before it can be selected.

## Exact source-only projection

Each complete candidate is compared with the registered t=74 reference after
exact enumeration of the current delivery outcomes. It is eligible only if
all conditions below hold:

- expected posterior-summary disagreement decreases by at least 0.25%;
- reference-weighted existence-retention risk is at most 0.01;
- no formation loses more than 0.05 expected cardinality on average;
- every reference-supported label retains at least 80% of its expected
  existence probability;
- no label expected above 0.5 under reference falls below 0.5;
- the physical graph, dominant sources, formation-level cycle, residual
  weight, and directed message count exactly match the registered contract;
- the selected t=72--t=74 topology and two registered recovery steps pass
  rolling-B3 connectivity at both sensor and formation levels.

Among eligible routes, the controller selects the route with the largest
exact one-round disagreement reduction. If none is eligible, it returns the
registered reference. This projector is deliberately stricter than the
gateway proposal and remains deterministic even if a learned ranker is added
later.

## Frozen evidence boundary

The first preflight may replay only the already-opened M24 seed-211 V30
trajectory to recover the causal pre-fusion state at t=74. It may evaluate at
most 16 reference-anchored gateway cycles plus reference, and it must not
score a tracking outcome. Only a clean, committed preflight that selects a safe
nonreference route above the 0.25% disagreement margin authorizes one paired
rerun of the already-opened t=72--t=74 outcome. Failure stops V32 without
opening another M24 state, training a GNN, or running X36/X48.

Even a successful paired rerun would remain development evidence. It would
show that sensor-level gateway choice can repair the forced reconnect under a
fixed communication and formation-graph budget; it would not establish
multi-state or cross-scale generalization.
