# V162 observable one-hop risk labels: X36 t72 H=8

- Matched baseline: `matched-static-fixed-ccw-full-payload`
- Frozen H=8 reference reused: `0`

| Arm | Mean E-OSPA | E-OSPA gain | Mean RMSE | RMSE gain | Bytes saving |
|:--|--:|--:|--:|--:|--:|
| Static full payload | 84.037151 | -- | 59.967347 | -- | -- |
| V162 observable risk Top-4 | 76.322368 | +9.180% | 55.219073 | +7.918% | +2.458% |

| t | Static | Candidate | Gain | Protected formations |
|--:|--:|--:|--:|:--|
| 72 | 86.118620 | 85.071354 | +1.216% | [1 2 4 5] |
| 73 | 85.408155 | 83.601644 | +2.115% | [1 2 3 4 5] |
| 74 | 86.384056 | 82.011795 | +5.061% | [1 2 3 4 5] |
| 75 | 85.605271 | 80.345140 | +6.145% | [1 2 3 4 5 6] |
| 76 | 82.342302 | 76.089493 | +7.594% | [1 2 3 4 5 6] |
| 77 | 81.628263 | 71.294926 | +12.659% | [1 2 3 4 5 6] |
| 78 | 83.556650 | 67.966513 | +18.658% | [1 2 3 4 5 6] |
| 79 | 81.253892 | 64.198082 | +20.991% | [1 2 3 4 5 6] |

- Formation gains: `[13.82 7.76 7.711 8.97 11.25 5.915]%`
- Minimum after maturity: `+6.145%`
- F6 non-gateway terminal gain: `+21.194%`
- Worst sensor / minimum formation: `+19.320% / +5.915%`
- RMSE mean / worst sensor / minimum formation: `+7.918% / +10.248% / -29.799%`
- Minimum formation-time gain: `+0.000%`
- Window / terminal consensus: `+11.509% / +16.402%`
- Static / candidate runtime: `250.92 / 259.77 s`
- Auxiliary-state maintenance cost included: `1`
- V162 trigger cells by time: `[0 0 0 0 6 6 12 12]`
- V162 applied cells by time: `[0 0 0 0 6 6 12 12]`
- V162 selected labels by time: `[0 0 0 0 24 24 48 48]`
- V162 applied labels by time: `[0 0 0 0 22 21 43 43]`
- V162 attempted synopsis / request / response bytes by time: `[0 0 0 0 41248 41944 84304 84216] / [0 0 0 0 576 576 1152 1040] / [0 0 0 0 75152 71736 146888 146696]`
- V162 delivered synopsis / request / response bytes by time: `[0 0 0 0 33104 35192 73936 72528] / [0 0 0 0 528 504 1032 936] / [0 0 0 0 75152 71736 146888 146696]`
- V162 control and response bytes are already included in candidate attempted bytes.
- Auxiliary runtime included / memory quantified: `1 / 1`
- Registered gate passed: `0`

## Evidence boundary

V162 is a paired recursive X36 seed-211 t=72 H=8 development screen. It retains the frozen V105 base route, fusion weights and protection schedule, and reuses the 36 opened receiver-time cells only as a trigger schedule. At each cell, current physical neighbors advertise present-time label keys and scalar posterior Bayes risks. The receiver chooses the minimum-risk one-hop source per label and requests at most four labels with positive receiver-minus-source risk. Responses carry complete Bernoulli Gaussian-mixture densities. The filter ledger charges synopsis, request, response-header and complete-label bytes on their directed physical links; all messages reuse the frozen per-link delivery realization and do not perturb the base communication RNG. Truth and future measurements do not enter any source or label decision. The opened trigger schedule remains privileged, so V162 is not an online method, validation or generalization evidence.
