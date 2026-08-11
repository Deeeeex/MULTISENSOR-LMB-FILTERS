# V97 positive-net addressable payload

- Matched baseline: `matched-static-fixed-ccw-full-payload`
- Receiver semantics: `fov-aware-censored`
- Pairing: same cached posterior, measurements, link uniforms, filter RNG, static carrier graph, fusion weights, horizon and communication constraints.

| Scale | t | V96 set | V97 set | Static | V96 | V97 | V97/static | V97/V96 | Worst sensor | Min formation | Consensus | Bytes | Pass |
|:--|--:|:--|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|:--:|
| M24 | 104 | [1 3] | [1 3 4] | 71.664511 | 67.229679 | 65.770429 | +8.225% | +2.171% | +18.852% | +0.000% | +18.576% | +3.950% | 1 |
| M24 | 124 | [2 3 4] | [1 2 3 4] | 83.582917 | 77.559560 | 76.841448 | +8.066% | +0.926% | +24.759% | +3.604% | +16.267% | +6.452% | 1 |
| X36 | 72 | [1 2 4] | [1 2 4 5] | 85.970277 | 84.623645 | 83.896827 | +2.412% | +0.859% | +7.129% | +0.000% | +4.273% | +5.305% | 0 |
| X36 | 100 | [4 5 6] | [1 3 4 5 6] | 89.375579 | 84.946782 | 82.964034 | +7.174% | +2.334% | +6.467% | +0.000% | +12.011% | +6.127% | 1 |

- Cross-scale pass: `3 / 4`; gate `0`.
- Full-episode tracking authorized: `0`

## Evidence boundary

V97 selects every currently addressable receiver formation whose observable rescue mass strictly exceeds its cross-supported useful-loss mass. Addressability requires an available action and zero cross-supported downward decision crossings. This is the exact positive-net set under the additive current-state surrogate and uses no tuned coverage target or fixed formation count. V96, V97 and reference arms share the same static carrier graph, cached posterior, measurements, link uniforms, filter RNG, horizon and communication constraints. These opened anchors are development evidence only.
