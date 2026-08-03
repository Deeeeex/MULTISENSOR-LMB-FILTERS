# Formation B4 V45 causal-route failure audit

## Scope and claim boundary

This record covers route executability only. It does not read target truth,
measurements, posterior contents, realized delivery uniforms, or tracking
scores. It therefore supports no claim about tracking accuracy, consensus
quality, or end-to-end communication savings.

The exploratory full-time run used the registered V43 development matrix:
M24 and X36, radial/convoy/relay/crossing, and seeds 41/43/47/53. The route
received one physical/link page at a time, but the first exploratory run still
used the legacy static graph as its formation-pair registry. Because the
function failed before returning, no aggregate MAT artifact was written; the
fail-fast log is retained under the ignored evidence directory.

## Reproducible failure

Cases 1--28 completed all 160 time steps. Case 29 stopped at:

| Field | Value |
|---|---|
| Preset | `x36-formation-fov-crossing` |
| Seed | 41 |
| Time | 158 |
| Error | `IndexEquivariantFormationRoute:NoCrossAssignment` |
| First failing constraint | A registered formation pair had no current physical sensor edge |

This is not a disconnected-scene failure. The current formation quotient graph
remains connected throughout times 157--160, while the t=1 registered tree
loses an increasing number of physical pairs:

| Time | Current quotient edges | Infeasible registered-tree edges | Hall-feasible current spanning trees |
|---:|---:|---:|---:|
| 157 | 11 | 0 | 225 |
| 158 | 10 | 1 | 130 |
| 159 | 9 | 2 | 66 |
| 160 | 8 | 3 | 32 |

All enumerated current spanning trees at these four pages admit distinct local
receivers for their incoming formation slots. Replacing the infeasible pair at
time 158 with one current cross-cut edge lets the unchanged V43 sensor-level
assignment construct a physical, strongly connected route with exactly
`2N = 72` directed messages. The failure is therefore caused by treating the
t=1 formation tree as a permanent hard constraint, not by the scene being
physically disconnected and not by the sensor-level assignment algorithm.

The same registered-pair loss occurs at the end of every registered X36
crossing development seed:

| Seed | First loss | Number of affected steps | Quotient disconnection steps |
|---:|---:|---:|---:|
| 41 | 158 | 3 | 0 |
| 43 | 157 | 4 | 0 |
| 47 | 158 | 3 | 0 |
| 53 | 157 | 4 | 0 |

## Causal-registry correction

`FormationB4V45BuildCausalRegisteredBaseGraph` removes the legacy dependency on
full-horizon intersection graphs and mean distances. A named formation ring is
persisted as an exogenous group-order-to-physical-UID pair registry. An initial
geometry tree is built only from time-1 positions and the time-1 physical page.
The sensor-level base graph is explicitly a pair-registry placeholder, not a
claim that those placeholder endpoints are executable communication links.

The registry identity hash is computed from physical-UID-canonical content and
is invariant to sensor-array permutation. A separate execution-view hash binds
the array-ordered placeholder used by the filter. The preflight also requires
contiguous audited times, exact adjacency/positive-weight support agreement,
the frozen V43/B4 row-weight patterns, actual rolling-B4 connectivity, and
exact 37.5% attempted-message saving over aligned four-step cycles.

The test suite intentionally preserves the time-158 X36 crossing failure as a
negative regression. This prevents a future implementation from silently
falling back to a stale or nonphysical registered edge.

## Decision

V45 is rejected as a full multistyle method: its fixed registered formation
tree is not causally executable on all registered pages. No formal full32 rerun
and no fresh tracking holdout should be opened for this method ID.

The next method version must insert a causal minimal-edit formation-backbone
projection before the existing V43 sensor assignment. The registered graph is
retained whenever feasible; otherwise the projection must use only the current
physical/reliability page, certify connectivity and distinct-receiver
matching, and fail closed if no feasible current backbone exists. This is a
material algorithm change and must use a new method ID and fresh structural
evidence.
