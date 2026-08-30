# V180 recursive V179-rollout capture: X36 t72 H=8

- Matched baseline: `matched-static-fixed-ccw-full-payload`
- Frozen H=8 reference reused: `1`

| Arm | Mean E-OSPA | E-OSPA gain | Mean RMSE | RMSE gain | Bytes saving |
|:--|--:|--:|--:|--:|--:|
| Static full payload | 84.037151 | -- | 59.967347 | -- | -- |
| V179 analytic F3 + rollout-aware learned F5 repair | 74.573180 | +11.262% | 53.566866 | +10.673% | -0.027% |

| t | Static | Candidate | Gain | Protected formations |
|--:|--:|--:|--:|:--|
| 72 | 86.118620 | 85.071354 | +1.216% | [1 2 4 5] |
| 73 | 85.408155 | 83.601644 | +2.115% | [1 2 3 4 5] |
| 74 | 86.384056 | 82.011795 | +5.061% | [1 2 3 4 5] |
| 75 | 85.605271 | 80.345140 | +6.145% | [1 2 3 4 5 6] |
| 76 | 82.342302 | 73.916504 | +10.233% | [1 2 3 4 5 6] |
| 77 | 81.628263 | 70.177270 | +14.028% | [1 2 3 4 5 6] |
| 78 | 83.556650 | 62.822109 | +24.815% | [1 2 3 4 5 6] |
| 79 | 81.253892 | 58.639621 | +27.832% | [1 2 3 4 5 6] |

- Formation gains: `[13.82 7.76 18.8 8.97 13.18 6.112]%`
- Minimum after maturity: `+6.145%`
- F6 non-gateway terminal gain: `+23.299%`
- Worst sensor / minimum formation: `+19.320% / +6.112%`
- RMSE mean / worst sensor / minimum formation: `+10.673% / +10.248% / -2.351%`
- Minimum formation-time gain: `+0.000%`
- Window / terminal consensus: `+9.823% / +13.868%`
- Static / candidate runtime: `254.79 / 799.41 s`
- Auxiliary-state maintenance cost included: `1`
- V162 trigger cells by time: `[0 0 0 0 12 6 24 24]`
- V162 applied cells by time: `[0 0 0 0 12 6 22 22]`
- V162 selected labels by time: `[0 0 0 0 48 24 78 76]`
- V162 applied labels by time: `[0 0 0 0 45 21 67 67]`
- V162 attempted synopsis / request / response bytes by time: `[0 0 0 0 82464 41896 243736 244576] / [0 0 0 0 1088 576 1872 1680] / [0 0 0 0 151576 71736 228808 216488]`
- V162 delivered synopsis / request / response bytes by time: `[0 0 0 0 70272 35144 220256 221368] / [0 0 0 0 1016 504 1608 1496] / [0 0 0 0 151576 71736 228808 216488]`
- V162 control and response bytes are already included in candidate attempted bytes.
- Auxiliary runtime included / memory quantified: `1 / 1`
- Registered gate passed: `0`

## Evidence boundary

V180 reruns only the frozen V179 candidate arm and captures its pre-side-channel fused posterior at each return time. The completed V179 static reference outcome is reused without recomputation. Inference and communication accounting are unchanged from V179; the additional snapshots are development instrumentation for a second same-seed policy-iteration round, not validation evidence.
