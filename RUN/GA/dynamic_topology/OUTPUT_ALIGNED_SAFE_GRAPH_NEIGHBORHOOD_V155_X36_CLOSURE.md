# V155 X36 output-aligned safe graph-neighborhood closure

Date: 2026-08-30

## Question

V152--V154 compared only a small hand-designed graph codebook. V155 asks the narrower question that those experiments could not answer: did the X36 result fail because graph-only actions have little tracking value, or because the proposal set missed a useful graph near the static reference?

Stage A therefore enumerates the complete radius-one neighborhood around the better static clockwise graph at seed 83 and time 60. Each candidate changes the residual-cycle cut in exactly one formation, keeps the dominant backbone, uses the same mixture-aware LMB-KLA weights, is held for all eight continuation pages, and executes exactly 60 directed posterior transmissions per page. Candidate construction uses neither truth nor future outcomes.

## Result

The frozen finalizer selected the static clockwise reference. All 18 candidate graphs were distinct, held for the complete horizon, and passed the V155 structural gate, but none passed the joint output gate.

| Quantity | Result |
|---|---:|
| Retained candidates | 0 / 18 |
| Best mean E-OSPA gain | +0.381% (candidate 3) |
| Candidate 3 worst-sensor gain | -0.017% |
| Candidate 3 minimum-formation gain | -0.254% |
| Candidate 3 consensus gain | +0.331% |
| Candidate 3 attempted-byte saving | -0.064% |
| Registered material mean-gain gate | +5.000% |
| Radius-two follow-up authorized | no |

The effect becomes less favorable as the changed cut moves downstream through the formation cycle:

| Changed formation | Candidates | Mean-gain range | Minimum-formation-gain range |
|---:|---:|---:|---:|
| 1 | 1--3 | +0.222% to +0.381% | -0.720% to -0.254% |
| 2 | 4--6 | -0.253% to +0.093% | -1.280% to -0.303% |
| 3 | 7--9 | -0.419% to +0.087% | -2.233% to -2.015% |
| 4 | 10--12 | -0.303% to -0.131% | -1.298% to -0.472% |
| 5 | 13--15 | -1.997% to -0.705% | -9.366% to -3.952% |
| 6 | 16--18 | -1.231% to -0.475% | -7.094% to -4.929% |

Candidate 1 illustrates the mechanism: changing the first formation cut improves formation 1 by 2.715% but degrades formation 2 by 0.720%, while formations 3--6 are effectively unchanged. The local graph change redistributes error along the information-flow cycle; it does not create a stable network-wide improvement. Across all 18 candidates, attempted-byte saving ranges only from -0.937% to +0.562%, so the rejection is driven by tracking and consensus rather than the 5% byte ceiling.

The generic raw field `directedMessageCountExactFraction` is not the V155 budget check: it retains an older `N`-message diagnostic convention. V155's registered structure gate directly checks `meanDirectedMessageCount == 2N-2F == 60`; all candidates pass it.

## Decision

Do not run radius two, M24 replication, multiseed confirmation, richer-scene transfer, or GNN training for this graph-only action family. Radius two was registered only for the case where at least one radius-one graph was admissible but below the 5% materiality gate; V155 found no admissible radius-one graph.

Together with the earlier multi-gateway, structural-contraction, codebook, and dwell experiments, V155 closes the current **topology-only, fixed-weight, full-posterior** route on X36. This is not a claim that dynamic communication is useless. It is a narrower result: rearranging which neighbor receives the same full posterior, under the same message opportunity budget and fixed KLA weights, does not expose sufficient stable tracking headroom in the tested X36 state.

The next method should therefore change the information action, not merely the complete graph. A defensible next question is whether the budget currently spent on redundant label information can be reassigned to carry complete, spatially supported label densities across the formations that lack them, while preserving exact per-label KLA semantics and deterministic per-label connectivity. That direction must be deduplicated against the existing label-omission and source-posterior experiments before a new protocol is opened.

## Evidence

- Final report: `RUN/GA/dynamic_topology/evidence/tracking_aligned_v155/safe_graph_neighborhood/x36_formation_fov/seed83/OUTPUT_ALIGNED_SAFE_GRAPH_NEIGHBORHOOD_V155_PILOT.md`
- Candidate execution commit: `551ea39`
- Static-reference source: V152 X36 seed-83 shards
- Window: 60:67
- Full candidate count: 18
