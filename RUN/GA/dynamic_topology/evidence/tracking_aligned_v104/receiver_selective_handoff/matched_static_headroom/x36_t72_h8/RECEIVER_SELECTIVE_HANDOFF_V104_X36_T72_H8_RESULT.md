# V104 receiver-selective oracle: X36 t72 H=8

- Matched baseline: `matched-static-fixed-ccw-full-payload`
- Frozen H=8 reference reused: `1`

| Arm | Mean E-OSPA | Gain vs static | Bytes saving |
|:--|--:|--:|--:|
| Static full payload | 84.037151 | -- | -- |
| V104 receiver oracle | 79.555155 | +5.333% | +6.091% |

| t | Static | V104 | Gain | Selected receivers |
|--:|--:|--:|--:|:--|
| 72 | 86.118620 | 85.071354 | +1.216% | [] |
| 73 | 85.408155 | 83.601644 | +2.115% | [] |
| 74 | 86.384056 | 82.011795 | +5.061% | [] |
| 75 | 85.605271 | 79.999545 | +6.548% | [9 10 22 28 29 30] |
| 76 | 82.342302 | 77.918959 | +5.372% | [] |
| 77 | 81.628263 | 75.093820 | +8.005% | 17 |
| 78 | 83.556650 | 77.175159 | +7.637% | [] |
| 79 | 81.253892 | 75.568962 | +6.997% | [34 36] |

- Formation gains: `[-0.9312 4.934 7.716 8.961 11.58 -0.02167]%`
- Minimum post-handoff gain: `+5.372%`
- F6 non-gateway terminal gain: `-2.945%`
- Worst sensor / minimum formation: `+16.734% / -0.931%`
- Window / terminal consensus: `+9.591% / +17.077%`
- Static / V104 runtime: `251.51 / 243.15 s`
- Registered gate passed: `0`

## Evidence boundary

V104 is a retrospective receiver-granularity headroom oracle. It keeps only V103 handoff receiver rows whose same-page paired E-OSPA gain was strictly positive; target truth and the opened V103 outcome therefore select the frozen receiver set. It is not deployable and cannot support a validation claim. The V103 protection schedule, maturity timing, physical carrier, row message counts, positive-weight multisets, cached inputs, filter randomness and H=8 horizon are unchanged. The already frozen H=8 static outcome is reused only after preset, seed, receiver mode, horizon, cache path and cache SHA-256 match. V104 tests whether receiver granularity alone can remove the V103 local regressions before opening a label-wise action space.
