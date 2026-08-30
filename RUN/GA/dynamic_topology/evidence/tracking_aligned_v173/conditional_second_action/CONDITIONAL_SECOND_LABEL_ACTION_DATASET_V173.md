# V173 conditional second-label action dataset

- Preset / seed: `x36-formation-fov / 211`
- Feature count: `50`
- Candidate rows / cells: `3282 / 30`

| Split | Cells | Rows | Joint-positive rows | Cells with a safe second label |
|:--|--:|--:|--:|--:|
| training | 18 | 2000 | 583 | 18 |
| calibration | 6 | 645 | 194 | 6 |
| heldout | 6 | 637 | 214 | 6 |

| t | F | Receiver | First E/R | Candidates | Safe | Safe labels |
|--:|--:|--:|:--|--:|--:|--:|
| 76 | 3 | 13 | +1.191/+6.606 | 106 | 27 | 12 |
| 76 | 3 | 14 | +1.304/+6.220 | 105 | 24 | 13 |
| 76 | 3 | 15 | +1.057/+0.594 | 106 | 37 | 15 |
| 76 | 3 | 16 | +1.154/+6.895 | 105 | 23 | 12 |
| 76 | 3 | 17 | +1.150/+6.401 | 109 | 26 | 14 |
| 76 | 3 | 18 | +1.144/+6.522 | 109 | 21 | 12 |
| 78 | 3 | 13 | +7.760/+0.424 | 110 | 34 | 16 |
| 78 | 3 | 14 | +6.969/+0.402 | 113 | 30 | 17 |
| 78 | 3 | 15 | +6.971/+140.221 | 113 | 34 | 16 |
| 78 | 3 | 16 | +6.971/+0.439 | 113 | 37 | 18 |
| 78 | 3 | 17 | +6.974/+0.435 | 112 | 36 | 17 |
| 78 | 3 | 18 | +6.965/+0.434 | 112 | 33 | 19 |
| 78 | 5 | 25 | +7.151/+0.251 | 115 | 32 | 15 |
| 78 | 5 | 26 | +7.143/+0.197 | 116 | 36 | 17 |
| 78 | 5 | 27 | +7.153/+0.240 | 116 | 39 | 19 |
| 78 | 5 | 28 | +7.151/+0.248 | 114 | 34 | 17 |
| 78 | 5 | 29 | +7.140/+0.267 | 115 | 42 | 18 |
| 78 | 5 | 30 | +7.147/+0.256 | 111 | 38 | 19 |
| 79 | 3 | 13 | +1.559/+9.911 | 109 | 25 | 15 |
| 79 | 3 | 14 | +6.948/+0.396 | 110 | 28 | 15 |
| 79 | 3 | 15 | +6.942/+0.392 | 107 | 37 | 17 |
| 79 | 3 | 16 | +6.944/+145.743 | 106 | 37 | 17 |
| 79 | 3 | 17 | +6.946/+0.398 | 105 | 33 | 16 |
| 79 | 3 | 18 | +6.942/+0.403 | 108 | 34 | 14 |
| 79 | 5 | 25 | +7.146/+0.137 | 107 | 29 | 13 |
| 79 | 5 | 26 | +7.145/+0.249 | 106 | 36 | 15 |
| 79 | 5 | 27 | +7.145/+0.255 | 106 | 37 | 16 |
| 79 | 5 | 28 | +7.147/+0.236 | 105 | 33 | 17 |
| 79 | 5 | 29 | +7.139/+0.248 | 107 | 40 | 21 |
| 79 | 5 | 30 | +7.147/+0.233 | 106 | 39 | 18 |

## Evidence boundary

V173 is an opened X36 seed-211 conditional-action dataset. The frozen compact V170 classifier selects a first complete label using only present observable features. Candidate second actions use the updated receiver posterior plus truth-free LMB cardinality, first-action and topology summaries; numeric label IDs are not features. Current truth scores the incremental second-action E-OSPA and RMSE gains. Splits remain grouped exactly as V166, so the same-seed heldout screen is a learnability test rather than validation.
