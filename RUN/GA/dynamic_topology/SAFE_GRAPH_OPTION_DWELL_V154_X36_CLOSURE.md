# V154 X36 closure: dwell does not repair a single-gateway carrier

## Decision

V154 is closed before M24 replication, multi-seed expansion, richer-scene
evaluation or GNN training.  On the opened `x36-formation-fov` seed-83
continuation, none of the six formation-scale graph options passes the frozen
tracking--tail--consensus--byte gate.  The best mean candidate, rank 5,
improves mean E-OSPA by only `0.248%`; it worsens the worst sensor by `0.014%`,
the weakest formation by `4.235%`, consensus by `1.926%`, and attempted bytes
by `0.887%`.  The result is far below the registered `5%` mean-gain threshold.

Every arm passes the schedule and graph-structure checks and changes its route
only once, at the second option boundary.  The negative result is therefore a
task-value failure of the frozen action family, not a failure to execute the
dwell protocol.

## Frozen X36 result

- Preset / seed / analysis window: `x36-formation-fov / 83 / 60:67`.
- Formation-scale dwell: `6` pages.
- Better static reference: clockwise spliced residual cycle.
- Exact directed transmission opportunities per page: `60` for every arm.
- Full mixture-aware LMB posterior on every selected edge.
- Every arm passes physicality, message-count and strong-connectivity checks.

| Rank | Mean gain | Worst-sensor gain | Minimum-formation gain | Consensus gain | Byte saving | Route changes | Retained |
|--:|--:|--:|--:|--:|--:|--:|:--:|
| 1 | -2.119% | -1.742% | -4.103% | -3.225% | -1.276% | 1 | no |
| 2 | -1.824% | -1.742% | -7.223% | -3.047% | -1.818% | 1 | no |
| 3 | -1.740% | -1.742% | -4.249% | -2.776% | -1.155% | 1 | no |
| 4 | -1.368% | -1.742% | -6.532% | -2.540% | -1.478% | 1 | no |
| 5 | +0.248% | -0.014% | -4.235% | -1.926% | -0.887% | 1 | no |
| 6 | -1.817% | -0.016% | -9.094% | -1.880% | +0.224% | 1 | no |

Negative byte saving means that the recursive trajectory serializes more bytes
than the paired static reference despite using the same number of directed
transmission opportunities.

## What the comparison with V153 establishes

Holding a complete graph for a formation-scale interval slightly changes the
finite-horizon response, but it does not change the conclusion.  For the only
rank with a positive mean, dwell moves the mean gain from V153's `0.088%` to
V154's `0.248%`, while the weakest-formation loss remains essentially unchanged
(`-4.372%` versus `-4.235%`) and the consensus loss becomes larger (`-1.290%`
versus `-1.926%`).

The evidence rejects rapid graph switching as a sufficient explanation for
the X36 failure.  Temporal commitment cannot create useful information paths
that are absent from the graph family itself.

## Structural diagnosis

All V152--V154 candidates retain the same basic carrier: each formation has
one cross-formation residual gateway and the collapsed formation graph is one
directed ring.  Increasing from four to six formations therefore lengthens the
formation-level influence path without increasing cross-formation entry
capacity or path diversity.  Changing gateway identities, ranking edge-diverse
maps, or holding one such map longer leaves this bottleneck intact.

This diagnosis is narrower than a claim that X36 requires more messages.  The
next test must first ask whether the existing residual-message budget can be
rewired into several cross-formation entrances and shorter alternative paths.
Only if that exact-budget graph family has no tracking headroom should added
communication be considered.

## Relation to closed multi-path branches

V89 used a three-phase acquire--broadcast--reference controller that replaced
one residual sender per nominated receiver formation and temporarily promoted
the acquired posterior into local dominant slots.  V119 added one local,
one-page secondary provenance input to a single failing formation.  Neither
constructed a complete, persistent, sensor-level strongly connected graph
with multiple cross gateways per formation while preserving the total residual
message count.  That exact-budget graph-reconstruction question therefore
remains open.

## Successor boundary

The successor may change only the residual graph topology.  It must keep the
full LMB posterior, nominal KLA weights, total directed-message count, physical
reachability and deterministic safety projection.  Its first bounded screen
must expose X36 seed-83 offline headroom before any selector, M24 replication,
additional seed, richer scene or learning is run.

The admissible family should cut each formation's local residual cycle at more
than one location and use the released edges to create multiple cross-formation
gateways and edge-diverse formation paths.  The resulting sensor graph must
remain a single strongly connected residual carrier; merely adding a local
input or repeating the V89 handover cadence is outside this successor.

## Reporting boundary

This is opened-development negative evidence.  Its numeric results remain in
the repository experiment record and are not promoted to the main progress
document or manuscript.  The stable method decision is only that graph dwell
and the current single-gateway ring codebook are closed on X36.
