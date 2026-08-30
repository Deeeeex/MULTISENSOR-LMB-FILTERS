# V176 recursive-rollout capture: X36 t72 H=8

- Matched baseline: `matched-static-fixed-ccw-full-payload`
- Frozen H=8 reference reused: `1`

| Arm | Mean E-OSPA | E-OSPA gain | Mean RMSE | RMSE gain | Bytes saving |
|:--|--:|--:|--:|--:|--:|
| Static full payload | 84.037151 | -- | 59.967347 | -- | -- |
| V169 analytic F3 + sequential learned F5 repair | 74.653711 | +11.166% | 53.589941 | +10.635% | -0.071% |

| t | Static | Candidate | Gain | Protected formations |
|--:|--:|--:|--:|:--|
| 72 | 86.118620 | 85.071354 | +1.216% | [1 2 4 5] |
| 73 | 85.408155 | 83.601644 | +2.115% | [1 2 3 4 5] |
| 74 | 86.384056 | 82.011795 | +5.061% | [1 2 3 4 5] |
| 75 | 85.605271 | 80.345140 | +6.145% | [1 2 3 4 5 6] |
| 76 | 82.342302 | 73.916504 | +10.233% | [1 2 3 4 5 6] |
| 77 | 81.628263 | 70.177270 | +14.028% | [1 2 3 4 5 6] |
| 78 | 83.556650 | 63.217835 | +24.341% | [1 2 3 4 5 6] |
| 79 | 81.253892 | 58.888145 | +27.526% | [1 2 3 4 5 6] |

- Formation gains: `[13.82 7.76 18.81 8.97 12.57 6.112]%`
- Minimum after maturity: `+6.145%`
- F6 non-gateway terminal gain: `+23.299%`
- Worst sensor / minimum formation: `+19.320% / +6.112%`
- RMSE mean / worst sensor / minimum formation: `+10.635% / +10.248% / -3.552%`
- Minimum formation-time gain: `+0.000%`
- Window / terminal consensus: `+9.804% / +13.892%`
- Static / candidate runtime: `256.66 / 848.18 s`
- Auxiliary-state maintenance cost included: `1`
- V162 trigger cells by time: `[0 0 0 0 12 6 24 24]`
- V162 applied cells by time: `[0 0 0 0 12 6 23 24]`
- V162 selected labels by time: `[0 0 0 0 48 24 78 78]`
- V162 applied labels by time: `[0 0 0 0 45 21 68 69]`
- V162 attempted synopsis / request / response bytes by time: `[0 0 0 0 82464 41896 243736 244840] / [0 0 0 0 1088 576 1872 1744] / [0 0 0 0 151576 71736 231552 223320]`
- V162 delivered synopsis / request / response bytes by time: `[0 0 0 0 70272 35144 220256 221632] / [0 0 0 0 1016 504 1640 1560] / [0 0 0 0 151576 71736 231552 223320]`
- V162 control and response bytes are already included in candidate attempted bytes.
- Auxiliary runtime included / memory quantified: `1 / 1`
- Registered gate passed: `0`

## Evidence boundary

V169 is a paired recursive X36 seed-211 t=72 H=8 development screen. It retains the frozen V105 route, weights and protection schedule plus the truth-free V162 analytic transfers at F1/F2/F3/F6. At the opened F5 repair cells, a frozen compact classifier first scores a deterministic seven-criterion source shortlist. After the first delivered label updates the receiver posterior, a second request is allowed only under a frozen high-cardinality-pressure gate; a conditional classifier filters candidates and minimum source evidence quality breaks ties. The policy may abstain and requests at most two sequential complete Bernoulli GM labels. Inference reads no truth, future measurements or numeric label IDs. The ledger charges a 24-byte packed synopsis per advertised active source-label, both request headers and complete-label responses on the frozen directed delivery realization. The repair schedule and most training cells were selected after opening seed 211, so this is recursive method-development evidence, not validation or a generalization claim.
