# V46: causal minimal-edit formation-backbone projection

## Motivation

V45 treated the formation graph registered at the beginning of an episode as
a permanent hard constraint. The X36 crossing audit disproved that assumption:
the current formation-level physical graph remained connected, but one to
three registered tree edges became unavailable near the end of the episode.
The sensor-level V43 constructor then failed even though many alternative
current spanning trees were feasible.

V46 changes the role of the registered graph. It is a low-churn prior that is
kept exactly when it remains feasible, not an edge set that may override the
current physical graph.

## Current-page safe projection

Let (E_0) be the registered undirected formation graph and (Q_t) the
formation quotient of the current physical sensor graph. A projected graph
(B_t \subseteq Q_t) is executable only if:

1. (B_t) is connected.
2. Every neighbor slot of a receiving formation can be assigned to a distinct
   local receiver sensor with at least one current physical sender in that
   neighbor formation.
3. The downstream V43 construction produces one dominant and one residual
   input per sensor, uses only current physical edges, and remains strongly
   connected.

The projector enforces the first two conditions. The composed runtime enforces
the third immediately after projection. Local dual-cycle feasibility depends
on the current within-formation physical graph, not on which formation pairs
are retained, so this final check cannot change the topology ranking; it can
only accept the composition or fail closed.

The projection searches edit layers in the frozen order

\[
\left(
|E_0 \setminus B_t|,
|B_t \setminus E_0|
\right).
\]

The first layer containing a cross-backbone-feasible graph is the only layer
eligible for selection. For each candidate topology, the frozen V43
constructor first determines its per-receiving-formation cross assignment.
Candidates are then
ranked by the aggregate bottleneck reliability, total log reliability, total
distance, and finally a physical-UID key of those V43-realized assignments.
Array indices and numeric group labels are not tie breakers. Exactness thus
means exhaustive comparison of candidate topologies inside the bounded search
under the frozen V43 assignment rule; it does not mean joint re-optimization
of every sensor-level sender and receiver choice.

The exact search is bounded and fail-closed. Its intended regime is two to
eight formations with a sparse registered ring/tree and occasional physical
edge loss. A synthetic exhaustive oracle covers up to six formations. The
method must not be described as a polynomial-time solution to general
constrained network design.

The projection certificate covers only formation-level connectivity and the
distinct-receiver cross-assignment witness. Local dual-cycle feasibility,
strict 2N execution, physical support, positive weight support, and sensor-level
strong connectivity become certified only after composition with the unchanged
V43 constructor. The runtime and preflight therefore run and verify that
composition immediately; they fail closed if a local cycle is unavailable.

## Structural result

Assume each formation admits the two distinct local directed cycles required
by V43 and the V46 projection returns an admissible connected formation graph.
For every undirected formation edge, the matching witness provides one
physical cross message in each direction and assigns different neighbor slots
to different receivers. Replacing those receivers' local residual inputs with
the cross inputs therefore preserves exactly one residual input per sensor.

Consequently the sensor-level reference has:

- exactly (N) dominant and (N) residual off-diagonal inputs;
- exactly (2N) attempted directed messages per full reference page;
- positive support equal to the selected adjacency;
- current physical-edge support only; and
- strong connectivity, obtained from local directed cycles plus a connected
  bidirectional formation backbone.

These are structural statements. They do not imply tracking improvement.

## B4 communication schedule

The primary V46 candidate is synchronized B4. The dominant layer is attempted
every step, while the complete current residual layer is attempted once every
four steps with residual weight 0.20. Over an aligned four-step cycle this uses
(5N) rather than (8N) attempted messages, an exact 37.5% reduction relative
to the full (2N)-per-step reference.

The projected sender identity may change between steps. Therefore V46 may say
that each receiver's four-step residual mass is matched to the reference
weight, but it must not reuse a per-edge mass-equivalence claim. The old static
V44 contraction evidence is development motivation only; V46 needs a new
audit of the actual time-varying matrix products and every realized rolling-B4
union.

Formation-staggered B4 is not a primary V46 arm. Its four realized pages can be
generated from four different repaired backbones, so a same-snapshot four-page
check is insufficient. It remains deferred until a causal history-aware
projection filters candidates against the previous three executed pages.

## Data-driven extension boundary

A learned model may later score formation edges inside the first feasible edit
layer. It must not output the executed topology directly. Connectivity,
physical support, distinct-receiver matching, the (2N) contract, and rolling
window safety remain exact projection constraints. UID values are excluded
from learned features and are used only for deterministic final tie breaking.

Thus an inaccurate model may change which equally minimal safe repair is
chosen, but cannot authorize a nonphysical or disconnected route. Any learned
model, feature schema, and weights must be frozen before fresh tracking
holdout seeds are opened.

## Evidence gates before tracking

1. Reproduce X36 crossing seed 41 at time 158 with one registered edge removed,
   one current edge added, strict 72-message reference execution, and no V43
   `NoCrossAssignment` failure.
2. Prove no-op behavior: whenever (E_0) is feasible, V46 must reproduce the
   V45 route and weight hash exactly.
3. Compare the bounded search against a full enumeration oracle on synthetic
   cases with at most six formations.
4. Cover Hall-conflict alternatives, no-feasible-backbone failure, sensor-array
   permutations, formation storage permutations, group relabeling, and an X48
   degree-seven rejection case.
5. Run all 32 registered M24/X36 route cases for all 160 steps from a clean
   commit. Record every projection edit, route hash, rolling-B4 union, and
   aligned communication count.
6. Integrate the projection into the real directed filter and run a non-scoring
   eight-step smoke with physical-UID-paired delivery uniforms.
7. Only after these gates pass may a fresh paired tracking sentinel be opened.
