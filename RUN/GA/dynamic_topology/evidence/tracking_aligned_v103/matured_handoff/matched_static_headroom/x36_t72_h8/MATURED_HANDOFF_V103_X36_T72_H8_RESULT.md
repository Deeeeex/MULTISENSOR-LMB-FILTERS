# V103 matured handoff: X36 t72 H=8

- Matched baseline: `matched-static-fixed-ccw-full-payload`
- Gateways by formation: `[2 8 14 20 26 32]`

| Arm | Mean E-OSPA | Gain vs static | Bytes saving |
|:--|--:|--:|--:|
| Static full payload | 84.037151 | -- | -- |
| V103 matured handoff | 79.554740 | +5.334% | +5.981% |

| t | Static | V103 | Gain | Handoff formations |
|--:|--:|--:|--:|:--|
| 72 | 86.118620 | 85.071354 | +1.216% | [] |
| 73 | 85.408155 | 83.601644 | +2.115% | [] |
| 74 | 86.384056 | 82.011795 | +5.061% | [] |
| 75 | 85.605271 | 80.008241 | +6.538% | [1 2 4 5] |
| 76 | 82.342302 | 77.770177 | +5.553% | [] |
| 77 | 81.628263 | 75.235133 | +7.832% | 3 |
| 78 | 83.556650 | 77.171269 | +7.642% | [] |
| 79 | 81.253892 | 75.568309 | +6.997% | 6 |

- Formation gains: `[-0.9448 4.676 7.731 9.079 11.74 -0.02199]%`
- Minimum post-handoff gain: `+5.553%`
- F6 non-gateway terminal gain: `-2.948%` (sensors `[31 33 34 35 36]`)
- Worst sensor / minimum formation: `+16.734% / -0.945%`
- Window / terminal consensus: `+9.651% / +18.432%`
- Static / V103 runtime: `251.51 / 244.01 s`
- Registered gate passed: `0`

## Evidence boundary

V103 is a frozen matched-static causal headroom probe. It retains V101 protection for three complete fusion pages before a gateway posterior is eligible for one later within-formation handoff. Different formation activation times determine the handoff pages; reference-recovery pages separate all handoffs so rolling three-page sensor- and formation-level reachability is retained. Every changed row preserves its reference message count and positive-weight multiset. Static and candidate arms share the cached posterior, measurements, delivery uniforms, filter RNG, carrier graph, communication model and horizon. The schedule is frozen before V103 outcomes are opened and is not an online policy or validation claim.
