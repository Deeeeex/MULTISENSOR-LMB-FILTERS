# V187 formation-coordinator aggregation: X36 t72 H=8

- Matched baseline: `matched-static-fixed-ccw-full-payload`
- Frozen H=8 reference reused: `1`

| Arm | Mean E-OSPA | E-OSPA gain | Mean RMSE | RMSE gain | Bytes saving |
|:--|--:|--:|--:|--:|--:|
| Static full payload | 84.037151 | -- | 59.967347 | -- | -- |
| V187 analytic F3 + coordinator-aggregated F5 repair | 74.678760 | +11.136% | 53.540189 | +10.718% | +0.160% |

| t | Static | Candidate | Gain | Protected formations |
|--:|--:|--:|--:|:--|
| 72 | 86.118620 | 85.071354 | +1.216% | [1 2 4 5] |
| 73 | 85.408155 | 83.601644 | +2.115% | [1 2 3 4 5] |
| 74 | 86.384056 | 82.011795 | +5.061% | [1 2 3 4 5] |
| 75 | 85.605271 | 80.345140 | +6.145% | [1 2 3 4 5 6] |
| 76 | 82.342302 | 73.916504 | +10.233% | [1 2 3 4 5 6] |
| 77 | 81.628263 | 70.177270 | +14.028% | [1 2 3 4 5 6] |
| 78 | 83.556650 | 62.822109 | +24.815% | [1 2 3 4 5 6] |
| 79 | 81.253892 | 59.484264 | +26.792% | [1 2 3 4 5 6] |

- Formation gains: `[13.82 7.76 18.8 8.97 12.39 6.112]%`
- Minimum after maturity: `+6.145%`
- F6 non-gateway terminal gain: `+23.299%`
- Worst sensor / minimum formation: `+19.320% / +6.112%`
- RMSE mean / worst sensor / minimum formation: `+10.718% / +10.248% / -1.050%`
- Minimum formation-time gain: `+0.000%`
- Window / terminal consensus: `+9.834% / +13.959%`
- Static / candidate runtime: `254.79 / 606.07 s`
- Auxiliary-state maintenance cost included: `1`
- V162 trigger cells by time: `[0 0 0 0 12 6 24 24]`
- V162 applied cells by time: `[0 0 0 0 12 6 22 23]`
- V162 selected labels by time: `[0 0 0 0 48 24 78 78]`
- V162 applied labels by time: `[0 0 0 0 45 21 67 68]`
- V162 attempted synopsis / request / response bytes by time: `[0 0 0 0 82464 41896 243736 177536] / [0 0 0 0 1088 576 1872 8592] / [0 0 0 0 151576 71736 228808 223320]`
- V162 delivered synopsis / request / response bytes by time: `[0 0 0 0 70272 35144 220256 161608] / [0 0 0 0 1016 504 1608 8408] / [0 0 0 0 151576 71736 228808 219904]`
- V162 control and response bytes are already included in candidate attempted bytes.
- Auxiliary runtime included / memory quantified: `1 / 1`
- Registered gate passed: `0`

## Evidence boundary

V187 is a paired recursive X36 seed-211 t=72 H=8 development screen. It preserves the V179 route, protection schedule and t=78 per-receiver F5 selector. At t=79, one current-graph formation coordinator receives each source inventory once plus one local summary from each peer receiver. Every label summary is charged as 64 bytes and is reconstructed as an existence-weighted 2-D moment object with source-computed observation opportunity and complete payload size. The frozen V185 common-action rule then requests one complete Bernoulli GM label for the six receivers. Selection uses no truth, future measurements or numeric label values. The rule and preflight were developed on opened seed 211, so this is development evidence only.
