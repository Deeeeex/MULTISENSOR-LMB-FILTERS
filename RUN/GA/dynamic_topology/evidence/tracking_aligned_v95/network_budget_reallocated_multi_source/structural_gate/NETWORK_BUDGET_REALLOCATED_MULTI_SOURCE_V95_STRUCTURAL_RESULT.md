# V95 sender-budget reallocation structural gate

The deployable arm moves each 0.05 residual token to a new receiver without changing its sender. Therefore its per-sender message count and network message count must match the static physical-tree baseline exactly. No tracking truth is read here.

| Scale | Candidates | Selected / required | Target forms | Source forms | Cuts | Messages B/O/C | Sender parity | Instant strong | Candidate B3 | Fixed B3 | Gate |
|:--|--:|:--|--:|--:|--:|:--|:--:|:--:|:--:|:--:|:--:|
| M24 | 10 | 2 / 2 | 2 | 2 | 0 | 48 / 46 / 48 | 1 | 1 | 1 | 1 | 1 |
| X36 | 23 | 3 / 3 | 3 | 3 | 5 | 72 / 69 / 72 | 1 | 1 | 1 | 1 | 1 |

- Cross-scale structural gate: `1`
- Matched-static headroom authorized: `1`
- Tracking outcome read: `0`

## Evidence boundary

V95 uses one already-opened braided-handover state per scale. A residual 0.05 message token is moved from its registered donor receiver to a physically reachable target receiver while keeping the sender unchanged. The dominant 0.70 inputs never move. The deployable candidate must preserve every sender message count and the network message budget exactly. It is compared with the same current physical-tree static route, a donor-only causal ablation, and the same selected route held fixed for the full horizon. Selection reads only current LMB posteriors, current association support, current physical links, link probabilities and past topology. Truth, future measurements, realized future links and tracking outcomes are unavailable until all four arms are frozen. These anchors are development evidence only.
