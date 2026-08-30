# V105 protection-only: X36 t72 H=8

- Matched baseline: `matched-static-fixed-ccw-full-payload`
- Frozen H=8 reference reused: `1`

| Arm | Mean E-OSPA | E-OSPA gain | Mean RMSE | RMSE gain | Bytes saving |
|:--|--:|--:|--:|--:|--:|
| Static full payload | 84.037151 | -- | 59.967347 | -- | -- |
| V105 protection only | 79.617863 | +5.259% | 62.175112 | -3.682% | +6.117% |

| t | Static | Candidate | Gain | Protected formations |
|--:|--:|--:|--:|:--|
| 72 | 86.118620 | 85.071354 | +1.216% | [1 2 4 5] |
| 73 | 85.408155 | 83.601644 | +2.115% | [1 2 3 4 5] |
| 74 | 86.384056 | 82.011795 | +5.061% | [1 2 3 4 5] |
| 75 | 85.605271 | 80.345140 | +6.145% | [1 2 3 4 5 6] |
| 76 | 82.342302 | 78.070034 | +5.188% | [1 2 3 4 5 6] |
| 77 | 81.628263 | 75.100317 | +7.997% | [1 2 3 4 5 6] |
| 78 | 83.556650 | 77.174602 | +7.638% | [1 2 3 4 5 6] |
| 79 | 81.253892 | 75.568021 | +6.998% | [1 2 3 4 5 6] |

- Formation gains: `[-0.9312 4.805 7.711 8.97 11.25 -0.0212]%`
- Minimum after maturity: `+5.188%`
- F6 non-gateway terminal gain: `-2.940%`
- Worst sensor / minimum formation: `+16.734% / -0.931%`
- RMSE mean / worst sensor / minimum formation: `-3.682% / +10.248% / -32.406%`
- Minimum formation-time gain: `-15.754%`
- Window / terminal consensus: `+9.650% / +17.214%`
- Static / candidate runtime: `250.92 / 241.10 s`
- Auxiliary-state maintenance cost included: `1`
- Auxiliary runtime included / memory quantified: `1 / 1`
- Registered gate passed: `0`

## Evidence boundary

V105 is a frozen H=8 causal attribution arm. It executes the exact V103 formation-conditioned control-only protection schedule but keeps every topology adjacency and fusion-weight row equal to the matched static fixed-counter-clockwise route. No gateway handoff occurs. The already frozen H=8 static outcome is reused only after matching the preset, seed, receiver mode, horizon, cache path and cache SHA-256. Cached inputs, measurements, delivery uniforms, filter RNG and communication model are unchanged. V105 separates prolonged protection from handoff and is opened development evidence, not an online policy or validation claim.
