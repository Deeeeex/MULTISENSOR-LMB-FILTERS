# V128 independent local anchor: X36 t72 H=8

- Matched baseline: `matched-static-fixed-ccw-full-payload`
- Frozen H=8 reference reused: `1`

| Arm | Mean E-OSPA | Gain vs static | Bytes saving |
|:--|--:|--:|--:|
| Static full payload | 84.037151 | -- | -- |
| V128 independent local anchor | 80.942391 | +3.683% | +6.874% |

| t | Static | Candidate | Gain | Protected formations |
|--:|--:|--:|--:|:--|
| 72 | 86.118620 | 85.071354 | +1.216% | [1 2 4 5] |
| 73 | 85.408155 | 83.601644 | +2.115% | [1 2 3 4 5] |
| 74 | 86.384056 | 82.011795 | +5.061% | [1 2 3 4 5] |
| 75 | 85.605271 | 80.345140 | +6.145% | [1 2 3 4 5 6] |
| 76 | 82.342302 | 78.561415 | +4.592% | [1 2 3 4 5 6] |
| 77 | 81.628263 | 79.266923 | +2.893% | [1 2 3 4 5 6] |
| 78 | 83.556650 | 79.699672 | +4.616% | [1 2 3 4 5 6] |
| 79 | 81.253892 | 78.981186 | +2.797% | [1 2 3 4 5 6] |

- Formation gains: `[-3.005 -2.129 7.711 8.97 11.25 -0.02928]%`
- Minimum after maturity: `+2.797%`
- F6 non-gateway terminal gain: `-5.784%`
- Worst sensor / minimum formation: `+10.530% / -3.005%`
- Minimum formation-time gain: `-22.646%`
- Window / terminal consensus: `+4.585% / +3.968%`
- Static / candidate runtime: `251.51 / 386.05 s`
- Auxiliary-state maintenance cost included: `0`
- Additional posterior payload messages / bytes: `0 / 0`
- Auxiliary runtime included / memory quantified: `1 / 0`
- Anchor-maintained nodes by time: `[36 36 36 36 36 36 36 36]`
- Registered gate passed: `0`

## Evidence boundary

V128 is a paired X36 seed-211 t=72 H=8 independent-anchor attribution. It retains the complete V105 route, fusion weights, protection schedule and opened V126 rollback mask. Each node forks an anchor from the common t=72 local posterior and thereafter propagates it using only its own prediction and measurement update. Rollback reads that independent local anchor, which adds no posterior message or payload byte. Candidate runtime includes anchor filtering; additional anchor memory is not yet quantified. The rollback timing still comes from opened outcomes, so V128 tests state-source feasibility only and is not an online policy, validation, or generalization claim.
