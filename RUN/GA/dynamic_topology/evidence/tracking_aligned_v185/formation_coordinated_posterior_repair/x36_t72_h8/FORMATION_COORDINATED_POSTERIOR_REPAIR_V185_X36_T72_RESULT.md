# V185 formation-coordinated posterior repair: X36 t72 H=8

- Matched baseline: `matched-static-fixed-ccw-full-payload`
- Frozen H=8 reference reused: `1`

| Arm | Mean E-OSPA | E-OSPA gain | Mean RMSE | RMSE gain | Bytes saving |
|:--|--:|--:|--:|--:|--:|
| Static full payload | 84.037151 | -- | 59.967347 | -- | -- |
| V185 analytic F3 + formation-coordinated F5 repair | 74.813076 | +10.976% | 53.619306 | +10.586% | -0.050% |

| t | Static | Candidate | Gain | Protected formations |
|--:|--:|--:|--:|:--|
| 72 | 86.118620 | 85.071354 | +1.216% | [1 2 4 5] |
| 73 | 85.408155 | 83.601644 | +2.115% | [1 2 3 4 5] |
| 74 | 86.384056 | 82.011795 | +5.061% | [1 2 3 4 5] |
| 75 | 85.605271 | 80.345140 | +6.145% | [1 2 3 4 5 6] |
| 76 | 82.342302 | 73.916504 | +10.233% | [1 2 3 4 5 6] |
| 77 | 81.628263 | 70.177270 | +14.028% | [1 2 3 4 5 6] |
| 78 | 83.556650 | 63.402745 | +24.120% | [1 2 3 4 5 6] |
| 79 | 81.253892 | 59.978158 | +26.184% | [1 2 3 4 5 6] |

- Formation gains: `[13.81 7.76 18.96 8.97 11.29 6.069]%`
- Minimum after maturity: `+6.145%`
- F6 non-gateway terminal gain: `+22.848%`
- Worst sensor / minimum formation: `+19.320% / +6.069%`
- RMSE mean / worst sensor / minimum formation: `+10.586% / +10.248% / -0.628%`
- Minimum formation-time gain: `+0.000%`
- Window / terminal consensus: `+9.804% / +13.356%`
- Static / candidate runtime: `254.79 / 786.68 s`
- Auxiliary-state maintenance cost included: `1`
- V162 trigger cells by time: `[0 0 0 0 12 6 24 24]`
- V162 applied cells by time: `[0 0 0 0 12 6 24 23]`
- V162 selected labels by time: `[0 0 0 0 48 24 78 78]`
- V162 applied labels by time: `[0 0 0 0 45 21 69 71]`
- V162 attempted synopsis / request / response bytes by time: `[0 0 0 0 82464 41896 243736 244576] / [0 0 0 0 1088 576 1872 1792] / [0 0 0 0 151576 71736 235640 230216]`
- V162 delivered synopsis / request / response bytes by time: `[0 0 0 0 70272 35144 220256 221368] / [0 0 0 0 1016 504 1672 1632] / [0 0 0 0 151576 71736 235640 230216]`
- V162 control and response bytes are already included in candidate attempted bytes.
- Auxiliary runtime included / memory quantified: `1 / 1`
- Registered gate passed: `0`

## Evidence boundary

V185 is a paired recursive X36 seed-211 t=72 H=8 development screen. It preserves the V179 route, weights, protection schedule and truth-free analytic transfers outside F5. At F5 t=78/79, all six receivers first complete ordinary KLA fusion. A frozen V181 classifier and V184 formation rule then select at most one exact label-source pair shared by the formation. Selection uses current posterior features, a 0.60 all-node safety floor, four-node support coverage, receiver label presence and observable Bayes risk; truth, future measurements and numeric label identifiers are excluded. The action replaces V179 per-node F5 requests rather than adding a second edit. Synopsis, request and complete-label response bytes are charged on each physical link. All policy-iteration data come from opened seed 211, so this is development evidence only.
