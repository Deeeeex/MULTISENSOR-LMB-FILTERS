# V96 addressable-risk adaptive payload

- Matched baseline: `matched-static-fixed-ccw-full-payload`
- Receiver semantics: `fov-aware-censored`
- Pairing: same cached posterior, measurements, link uniforms, filter RNG, horizon and communication constraints.

| Scale | t | Selected | Static | One step | Persistent | Gain/static | Gain/one step | Worst sensor | Min formation | Consensus | Bytes | Pass |
|:--|--:|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|:--:|
| M24 | 104 | [1 3] | 71.664511 | 69.842132 | 67.229679 | +6.188% | +3.741% | +18.852% | +0.000% | +13.641% | +1.501% | 1 |
| M24 | 124 | [2 3 4] | 83.582917 | 79.834853 | 77.559560 | +7.206% | +2.850% | +16.422% | +0.000% | +14.298% | +3.870% | 1 |
| X36 | 72 | [1 2 4] | 85.970277 | 85.442705 | 84.623645 | +1.566% | +0.959% | +7.129% | +0.000% | +4.358% | +4.033% | 0 |
| X36 | 100 | [4 5 6] | 89.375579 | 87.173619 | 84.946782 | +4.955% | +2.554% | +4.292% | +0.000% | +5.148% | +3.646% | 0 |

- Cross-scale pass: `2 / 4`; gate `0`.
- Full-episode tracking authorized: `0`

## Evidence boundary

V96 replaces the scale-diluted whole-network risk floor with conditional coverage of safely addressable current risk. A formation is addressable only when withholding its complete cross-formation posterior causes no cross-supported downward decision crossing. The smallest guarded subset covering at least 80% of addressable rescue is frozen before tracking truth is read. Reference, one-step and persistent arms share the same static carrier graph, cached posterior, measurements, link uniforms, filter RNG and communication constraints. These opened anchors are development evidence only.
