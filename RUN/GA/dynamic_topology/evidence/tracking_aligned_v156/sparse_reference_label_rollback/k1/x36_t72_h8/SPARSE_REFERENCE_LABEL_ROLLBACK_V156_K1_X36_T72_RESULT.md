# V156 sparse reference-label rollback K=1: X36 t72 H=8

- Matched baseline: `matched-static-fixed-ccw-full-payload`
- Frozen H=8 reference reused: `1`

| Arm | Mean E-OSPA | Gain vs static | Bytes saving |
|:--|--:|--:|--:|
| Static full payload | 84.037151 | -- | -- |
| V156 sparse reference labels K=1 | 78.974672 | +6.024% | +6.003% |

- Adjusted byte saving after reference-label payload: `+5.845%`
| t | Static | Candidate | Gain | Protected formations |
|--:|--:|--:|--:|:--|
| 72 | 86.118620 | 85.071354 | +1.216% | [1 2 4 5] |
| 73 | 85.408155 | 83.601644 | +2.115% | [1 2 3 4 5] |
| 74 | 86.384056 | 82.011795 | +5.061% | [1 2 3 4 5] |
| 75 | 85.605271 | 80.345140 | +6.145% | [1 2 3 4 5 6] |
| 76 | 82.342302 | 78.214786 | +5.013% | [1 2 3 4 5 6] |
| 77 | 81.628263 | 74.249079 | +9.040% | [1 2 3 4 5 6] |
| 78 | 83.556650 | 75.664990 | +9.445% | [1 2 3 4 5 6] |
| 79 | 81.253892 | 72.638587 | +10.603% | [1 2 3 4 5 6] |

- Formation gains: `[3.467 4.848 7.711 8.97 11.25 0.2598]%`
- Minimum after maturity: `+5.013%`
- F6 non-gateway terminal gain: `+0.023%`
- Worst sensor / minimum formation: `+15.440% / +0.260%`
- Minimum formation-time gain: `-6.017%`
- Window / terminal consensus: `+9.682% / +17.635%`
- Static / candidate runtime: `251.51 / 243.53 s`
- Auxiliary-state maintenance cost included: `0`
- Additional posterior payload messages / bytes: `36 / 45192`
- Reference-label counts by time: `[0 0 0 0 6 6 12 12]`
- Reference-label bytes by time: `[0 0 0 0 7224 6720 14448 16800]`
- Anchor attempted bytes by time: `[0 0 0 0 0 0 0 0]`
- Anchor delivered bytes by time: `[0 0 0 0 0 0 0 0]`
- Auxiliary runtime included / memory quantified: `1 / 1`
- Anchor-maintained nodes by time: `[0 0 0 0 0 0 0 0]`
- Registered gate passed: `0`

## Evidence boundary

V156 K=1 is a privileged paired X36 seed-211 t=72 H=8 representational-sufficiency oracle. It retains the V105 static carrier, fusion weights and protection schedule. At the 36 frozen V126 rollback cells it compares the evolving working posterior with the paired static posterior, ranks labels without truth or future measurements, and copies at most 1 complete GM Bernoulli label states (including explicit tombstones). The paired reference and opened rollback cells are counterfactual information, so the arm is not deployable and cannot support validation or generalization claims. Communication accounting still contains only the V105 main path; reference-label transport is estimated separately.
