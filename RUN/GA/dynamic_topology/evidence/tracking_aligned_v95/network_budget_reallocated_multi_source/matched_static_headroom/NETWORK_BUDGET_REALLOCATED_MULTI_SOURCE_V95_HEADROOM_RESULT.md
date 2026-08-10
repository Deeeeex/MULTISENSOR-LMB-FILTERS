# V95 four-arm matched-static headroom

- Baseline: `matched-static-current-physical-tree-full-payload`
- Receiver semantics: `fov-aware-censored`
- Pairing: identical cached posterior, measurements, link uniforms, filter RNG and horizon.

| Scale | H | Static | Donor only | Dynamic | Fixed reallocation | Gain/static | Gain/fixed | Gain/donor | Worst sensor | Min formation | Consensus | Bytes | Pass |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|:--:|
| M24 | 5 | 125.596200 | 125.596211 | 125.595824 | 125.699438 | +0.000% | +0.082% | +0.000% | +0.000% | -0.000% | -0.199% | +0.000% | 0 |
| X36 | 7 | 131.503832 | 131.473215 | 131.171915 | 130.945110 | +0.252% | -0.173% | +0.229% | +0.305% | +0.000% | +0.514% | +0.754% | 0 |

- Cross-scale matched-static gate: `0`
- Full-episode tracking authorized: `0`

## Evidence boundary

V95 uses one already-opened braided-handover state per scale. A residual 0.05 message token is moved from its registered donor receiver to a physically reachable target receiver while keeping the sender unchanged. The dominant 0.70 inputs never move. The deployable candidate must preserve every sender message count and the network message budget exactly. It is compared with the same current physical-tree static route, a donor-only causal ablation, and the same selected route held fixed for the full horizon. Selection reads only current LMB posteriors, current association support, current physical links, link probabilities and past topology. Truth, future measurements, realized future links and tracking outcomes are unavailable until all four arms are frozen. These anchors are development evidence only.
