# V157 positive-value reference labels: X36 t72 H=8

- Matched baseline: `matched-static-fixed-ccw-full-payload`
- Frozen H=8 reference reused: `1`

| Arm | Mean E-OSPA | Gain vs static | Bytes saving |
|:--|--:|--:|--:|
| Static full payload | 84.037151 | -- | -- |
| V157 positive-value reference labels | 78.556281 | +6.522% | +5.877% |

- Adjusted byte saving after reference-label payload: `+5.184%`

| t | Static | Candidate | Gain | Protected formations |
|--:|--:|--:|--:|:--|
| 72 | 86.118620 | 85.071354 | +1.216% | [1 2 4 5] |
| 73 | 85.408155 | 83.601644 | +2.115% | [1 2 3 4 5] |
| 74 | 86.384056 | 82.011795 | +5.061% | [1 2 3 4 5] |
| 75 | 85.605271 | 80.345140 | +6.145% | [1 2 3 4 5 6] |
| 76 | 82.342302 | 76.277596 | +7.365% | [1 2 3 4 5 6] |
| 77 | 81.628263 | 73.486208 | +9.975% | [1 2 3 4 5 6] |
| 78 | 83.556650 | 74.933174 | +10.321% | [1 2 3 4 5 6] |
| 79 | 81.253892 | 72.723340 | +10.499% | [1 2 3 4 5 6] |

- Formation gains: `[5.376 5.64 7.711 8.97 11.25 0.5415]%`
- Minimum after maturity: `+6.145%`
- F6 non-gateway terminal gain: `+0.028%`
- Worst sensor / minimum formation: `+17.384% / +0.542%`
- Minimum formation-time gain: `+0.000%`
- Window / terminal consensus: `+10.163% / +14.668%`
- Static / candidate runtime: `251.51 / 418.12 s`
- Auxiliary-state maintenance cost included: `1`
- Additional posterior payload messages / bytes: `36 / 198144`
- Anchor attempted bytes by time: `[0 0 0 0 0 0 0 0]`
- Anchor delivered bytes by time: `[0 0 0 0 0 0 0 0]`
- Auxiliary runtime included / memory quantified: `1 / 1`
- Anchor-maintained nodes by time: `[0 0 0 0 0 0 0 0]`
- Registered gate passed: `1`

## Evidence boundary

V157 is a privileged paired X36 seed-211 t=72 H=8 label-value mechanism oracle. It retains the V105 carrier, fusion weights and protection schedule. At the 36 frozen V126 rollback cells it greedily tests complete GM Bernoulli reference-label edits against exact current-step truth E-OSPA, accepts only strictly positive marginal edits, and stops after at most four labels. The reference state, rollback cells and truth-valued selection are privileged, so V157 is not deployable and cannot support validation or generalization claims. Communication is conservatively charged 198144 bytes, equal to four maximum-size labels plus one header at every rollback cell.
