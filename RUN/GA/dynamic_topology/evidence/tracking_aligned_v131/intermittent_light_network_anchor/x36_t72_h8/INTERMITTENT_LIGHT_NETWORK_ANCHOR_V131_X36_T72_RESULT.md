# V131 intermittent light network anchor: X36 t72 H=8

- Matched baseline: `matched-static-fixed-ccw-full-payload`
- Frozen H=8 reference reused: `1`

| Arm | Mean E-OSPA | Gain vs static | Bytes saving |
|:--|--:|--:|--:|
| Static full payload | 84.037151 | -- | -- |
| V131 intermittent light network anchor | 78.843264 | +6.180% | +6.465% |

| t | Static | Candidate | Gain | Protected formations |
|--:|--:|--:|--:|:--|
| 72 | 86.118620 | 85.071354 | +1.216% | [1 2 4 5] |
| 73 | 85.408155 | 83.601644 | +2.115% | [1 2 3 4 5] |
| 74 | 86.384056 | 82.011795 | +5.061% | [1 2 3 4 5] |
| 75 | 85.605271 | 80.345140 | +6.145% | [1 2 3 4 5 6] |
| 76 | 82.342302 | 78.697760 | +4.426% | [1 2 3 4 5 6] |
| 77 | 81.628263 | 74.455055 | +8.788% | [1 2 3 4 5 6] |
| 78 | 83.556650 | 73.958994 | +11.486% | [1 2 3 4 5 6] |
| 79 | 81.253892 | 72.604371 | +10.645% | [1 2 3 4 5 6] |

- Formation gains: `[0.8239 7.169 7.711 8.97 11.25 1.244]%`
- Minimum after maturity: `+4.426%`
- F6 non-gateway terminal gain: `+3.149%`
- Worst sensor / minimum formation: `+18.134% / +0.824%`
- Minimum formation-time gain: `-9.649%`
- Window / terminal consensus: `+12.628% / +29.358%`
- Static / candidate runtime: `251.51 / 268.03 s`
- Auxiliary-state maintenance cost included: `0`
- Additional posterior payload messages / bytes: `240 / 1051200`
- Anchor attempted bytes by time: `[263232 0 259776 0 260736 0 267456 0]`
- Anchor delivered bytes by time: `[245632 0 255280 0 248016 0 258272 0]`
- Auxiliary runtime included / memory quantified: `1 / 0`
- Anchor-maintained nodes by time: `[36 36 36 36 36 36 36 36]`
- Registered gate passed: `0`

## Evidence boundary

V131 is a paired X36 seed-211 t=72 H=8 intermittent-anchor attribution. It retains the complete V105 working route, fusion weights and protection schedule, while maintaining a parallel network anchor at every node. The anchor uses local prediction and measurement updates on every page, but exchanges moment-compressed LMB posteriors only at t=72, 74, 76 and 78 over the same active edges and paired delivery outcomes. All auxiliary attempted and delivered bytes and runtime are charged directly; additional anchor memory is not yet byte-quantified. Rollback uses the opened V126 formation-time mask, so V131 remains a privileged mechanism test and cannot support online, validation, or generalization claims.
