# V156 sparse reference-label rollback K=8: X36 t72 H=8

- Matched baseline: `matched-static-fixed-ccw-full-payload`
- Frozen H=8 reference reused: `1`

| Arm | Mean E-OSPA | Gain vs static | Bytes saving |
|:--|--:|--:|--:|
| Static full payload | 84.037151 | -- | -- |
| V156 sparse reference labels K=8 | 79.103428 | +5.871% | +5.995% |

- Adjusted byte saving after reference-label payload: `+4.752%`
| t | Static | Candidate | Gain | Protected formations |
|--:|--:|--:|--:|:--|
| 72 | 86.118620 | 85.071354 | +1.216% | [1 2 4 5] |
| 73 | 85.408155 | 83.601644 | +2.115% | [1 2 3 4 5] |
| 74 | 86.384056 | 82.011795 | +5.061% | [1 2 3 4 5] |
| 75 | 85.605271 | 80.345140 | +6.145% | [1 2 3 4 5 6] |
| 76 | 82.342302 | 77.272148 | +6.157% | [1 2 3 4 5 6] |
| 77 | 81.628263 | 74.180724 | +9.124% | [1 2 3 4 5 6] |
| 78 | 83.556650 | 76.137504 | +8.879% | [1 2 3 4 5 6] |
| 79 | 81.253892 | 74.207111 | +8.673% | [1 2 3 4 5 6] |

- Formation gains: `[2.563 4.935 7.711 8.97 11.25 0.1253]%`
- Minimum after maturity: `+6.145%`
- F6 non-gateway terminal gain: `+0.023%`
- Worst sensor / minimum formation: `+15.343% / +0.125%`
- Minimum formation-time gain: `-0.091%`
- Window / terminal consensus: `+9.682% / +15.991%`
- Static / candidate runtime: `251.51 / 245.74 s`
- Auxiliary-state maintenance cost included: `0`
- Additional posterior payload messages / bytes: `36 / 355152`
- Reference-label counts by time: `[0 0 0 0 48 48 96 96]`
- Reference-label bytes by time: `[0 0 0 0 58800 59472 109368 127512]`
- Anchor attempted bytes by time: `[0 0 0 0 0 0 0 0]`
- Anchor delivered bytes by time: `[0 0 0 0 0 0 0 0]`
- Auxiliary runtime included / memory quantified: `1 / 1`
- Anchor-maintained nodes by time: `[0 0 0 0 0 0 0 0]`
- Registered gate passed: `0`

## Evidence boundary

V156 K=8 is a privileged paired X36 seed-211 t=72 H=8 representational-sufficiency oracle. It retains the V105 static carrier, fusion weights and protection schedule. At the 36 frozen V126 rollback cells it compares the evolving working posterior with the paired static posterior, ranks labels without truth or future measurements, and copies at most 8 complete GM Bernoulli label states (including explicit tombstones). The paired reference and opened rollback cells are counterfactual information, so the arm is not deployable and cannot support validation or generalization claims. Communication accounting still contains only the V105 main path; reference-label transport is estimated separately.
