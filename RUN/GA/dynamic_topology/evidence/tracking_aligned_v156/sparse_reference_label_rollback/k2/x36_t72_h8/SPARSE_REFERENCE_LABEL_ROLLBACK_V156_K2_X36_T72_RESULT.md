# V156 sparse reference-label rollback K=2: X36 t72 H=8

- Matched baseline: `matched-static-fixed-ccw-full-payload`
- Frozen H=8 reference reused: `1`

| Arm | Mean E-OSPA | Gain vs static | Bytes saving |
|:--|--:|--:|--:|
| Static full payload | 84.037151 | -- | -- |
| V156 sparse reference labels K=2 | 79.214945 | +5.738% | +6.149% |

- Adjusted byte saving after reference-label payload: `+5.856%`
| t | Static | Candidate | Gain | Protected formations |
|--:|--:|--:|--:|:--|
| 72 | 86.118620 | 85.071354 | +1.216% | [1 2 4 5] |
| 73 | 85.408155 | 83.601644 | +2.115% | [1 2 3 4 5] |
| 74 | 86.384056 | 82.011795 | +5.061% | [1 2 3 4 5] |
| 75 | 85.605271 | 80.345140 | +6.145% | [1 2 3 4 5 6] |
| 76 | 82.342302 | 77.594753 | +5.766% | [1 2 3 4 5 6] |
| 77 | 81.628263 | 74.334784 | +8.935% | [1 2 3 4 5 6] |
| 78 | 83.556650 | 76.737422 | +8.161% | [1 2 3 4 5 6] |
| 79 | 81.253892 | 74.022668 | +8.900% | [1 2 3 4 5 6] |

- Formation gains: `[3.011 3.789 7.711 8.97 11.25 0.1258]%`
- Minimum after maturity: `+5.766%`
- F6 non-gateway terminal gain: `+0.023%`
- Worst sensor / minimum formation: `+14.069% / +0.126%`
- Minimum formation-time gain: `-1.353%`
- Window / terminal consensus: `+8.923% / +15.836%`
- Static / candidate runtime: `251.51 / 243.96 s`
- Auxiliary-state maintenance cost included: `0`
- Additional posterior payload messages / bytes: `36 / 83688`
- Reference-label counts by time: `[0 0 0 0 12 12 24 24]`
- Reference-label bytes by time: `[0 0 0 0 13080 11400 25992 33216]`
- Anchor attempted bytes by time: `[0 0 0 0 0 0 0 0]`
- Anchor delivered bytes by time: `[0 0 0 0 0 0 0 0]`
- Auxiliary runtime included / memory quantified: `1 / 1`
- Anchor-maintained nodes by time: `[0 0 0 0 0 0 0 0]`
- Registered gate passed: `0`

## Evidence boundary

V156 K=2 is a privileged paired X36 seed-211 t=72 H=8 representational-sufficiency oracle. It retains the V105 static carrier, fusion weights and protection schedule. At the 36 frozen V126 rollback cells it compares the evolving working posterior with the paired static posterior, ranks labels without truth or future measurements, and copies at most 2 complete GM Bernoulli label states (including explicit tombstones). The paired reference and opened rollback cells are counterfactual information, so the arm is not deployable and cannot support validation or generalization claims. Communication accounting still contains only the V105 main path; reference-label transport is estimated separately.
