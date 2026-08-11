# V102 shield/broadcast: X36 t72 H=6

- Matched baseline: `matched-static-fixed-ccw-full-payload`
- Gateways by formation: `[2 8 14 20 26 32]`

| Arm | Mean E-OSPA | Gain vs static | Bytes saving |
|:--|--:|--:|--:|
| Static full payload | 84.581111 | -- | -- |
| V102 shield/broadcast | 80.733928 | +4.549% | +5.021% |

| t | Static | V102 | Gain | Broadcast formations |
|--:|--:|--:|--:|:--|
| 72 | 86.118620 | 85.071354 | +1.216% | [] |
| 73 | 85.408155 | 82.330473 | +3.603% | [1 2 4 5] |
| 74 | 86.384056 | 82.017574 | +5.055% | [] |
| 75 | 85.605271 | 80.498531 | +5.965% | [1 2 3 4 5] |
| 76 | 82.342302 | 77.744767 | +5.583% | [] |
| 77 | 81.628263 | 76.740866 | +5.987% | [1 2 3 4 5 6] |

- Formation gains: `[2.419 3.296 5.148 7.276 9.432 0.1649]%`
- Minimum post-propagation gain: `+5.583%`
- Worst sensor / minimum formation: `+13.668% / +0.165%`
- Window / terminal consensus: `+8.579% / +13.997%`
- Static / V102 runtime: `184.10 / 177.60 s`
- Registered gate passed: `0`

## Evidence boundary

V102 is a frozen matched-static headroom probe that composes V101 cross-formation control-only protection with a one-step-delayed within-formation broadcast. Broadcast and reference-recovery pages alternate so every rolling three-page window retains the registered sensor-level information-flow reserve. For every protected formation, the registered cross-input gateway becomes the 0.70 dominant sender of physically reachable peers on the next page; the displaced dominant sender is retained at 0.05 and the old residual is removed. Every row preserves its message count and positive-weight multiset. Static and candidate arms share the cached posterior, measurements, delivery uniforms, filter RNG and communication model. The schedule is frozen before V102 outcomes are opened and is not yet an online policy or validation claim.
