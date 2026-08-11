# V96 addressable-risk structural gate

No tracking truth is read. Every arm retains the same static carrier graph and fusion weights.

| Scale | t | Selected | Total risk | Addressable risk | Coverage | Loss / rescue | Graph | Weights | Schedule | Gate |
|:--|--:|:--|--:|--:|--:|--:|:--:|:--:|:--:|:--:|
| M24 | 104 | [1 3] | 3.499% | 3.442% | 83.906% | 0.002% | 1 | 1 | 1 | 1 |
| M24 | 124 | [2 3 4] | 1.619% | 1.619% | 82.332% | 0.008% | 1 | 1 | 1 | 1 |
| X36 | 72 | [1 2 4] | 0.907% | 0.752% | 88.433% | 15.532% | 1 | 1 | 1 | 1 |
| X36 | 100 | [4 5 6] | 0.964% | 0.647% | 90.161% | 0.071% | 1 | 1 | 1 | 1 |

- Cross-scale structural gate: `1`
- Tracking headroom authorized: `1`
- Tracking outcome read: `0`

## Evidence boundary

V96 replaces the scale-diluted whole-network risk floor with conditional coverage of safely addressable current risk. A formation is addressable only when withholding its complete cross-formation posterior causes no cross-supported downward decision crossing. The smallest guarded subset covering at least 80% of addressable rescue is frozen before tracking truth is read. Reference, one-step and persistent arms share the same static carrier graph, cached posterior, measurements, link uniforms, filter RNG and communication constraints. These opened anchors are development evidence only.
