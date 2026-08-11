# V106 protection-release oracle: X36 t72 H=8

- Matched baseline: `matched-static-fixed-ccw-full-payload`
- Frozen H=8 reference reused: `1`

| Arm | Mean E-OSPA | Gain vs static | Bytes saving |
|:--|--:|--:|--:|
| Static full payload | 84.037151 | -- | -- |
| V106 release oracle | 79.628292 | +5.246% | +5.286% |

| t | Static | V106 | Gain | Protected | Released |
|--:|--:|--:|--:|:--|:--|
| 72 | 86.118620 | 85.071354 | +1.216% | [1 2 4 5] | [] |
| 73 | 85.408155 | 83.601644 | +2.115% | [1 2 3 4 5] | [] |
| 74 | 86.384056 | 82.011795 | +5.061% | [1 2 3 4 5] | [] |
| 75 | 85.605271 | 80.345140 | +6.145% | [1 2 3 4 5 6] | [] |
| 76 | 82.342302 | 78.070034 | +5.188% | [1 2 3 4 5 6] | [] |
| 77 | 81.628263 | 75.909913 | +7.005% | [2 3 4 5 6] | 1 |
| 78 | 83.556650 | 77.472335 | +7.282% | [2 3 4 5] | [1 6] |
| 79 | 81.253892 | 74.544116 | +8.258% | [2 3 4 5] | [1 6] |

- Formation gains: `[-0.8652 4.805 7.711 8.97 11.25 -0.1564]%`
- Formation-by-time gains (rows F1--F6):

```text
[0 0 4.833 9.205 4.202 -12.18 -11 -5.681;2.578 1.533 5.969 6.172 -4.928 5.07 10.27 11.14;0 3.123 4.065 4.628 12.47 10.38 12.86 15.17;1.01 2.944 7.934 9.996 8.744 19.14 11.87 9.873;3.686 5.659 7.723 6.413 11.37 17.08 17.84 18.6;0 0 0 0.001593 0.001866 0.9796 -0.003256 -2.406]
```
- Minimum formation-time gain: `-12.181%`
- Minimum after maturity: `+5.188%`
- F6 non-gateway terminal gain: `-2.947%`
- Worst sensor / minimum formation: `+16.734% / -0.865%`
- Window / terminal consensus: `+9.148% / +19.198%`
- Static / V106 runtime: `251.51 / 243.97 s`
- Registered gate passed: `0`

## Evidence boundary

V106 is a frozen retrospective protection-release headroom oracle. It starts from the exact V105 control-only protection schedule and releases F1 at t=77 and F6 at t=78, immediately before their opened V105 formation or peer outcomes reverse sign. Truth and opened V105 outcomes therefore define the release pages; V106 is not deployable and cannot support validation or generalization claims. Every topology adjacency and fusion-weight row stays equal to the matched static fixed-counter-clockwise route, and no gateway handoff occurs. The frozen H=8 static outcome is reused only after preset, seed, receiver mode, horizon, cache path and cache SHA-256 match. Cached inputs, measurements, delivery uniforms, filter RNG and the communication model are unchanged. V106 tests whether timely release has enough strict headroom to justify a later causal debt controller.
