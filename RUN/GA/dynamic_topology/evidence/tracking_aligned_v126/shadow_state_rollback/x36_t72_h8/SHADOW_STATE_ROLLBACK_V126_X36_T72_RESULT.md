# V126 shadow-state rollback: X36 t72 H=8

- Matched baseline: `matched-static-fixed-ccw-full-payload`
- Frozen H=8 reference reused: `1`

| Arm | Mean E-OSPA | Gain vs static | Bytes saving |
|:--|--:|--:|--:|
| Static full payload | 84.037151 | -- | -- |
| V126 shadow-state rollback | 79.242997 | +5.705% | +6.032% |

| t | Static | Candidate | Gain | Protected formations |
|--:|--:|--:|--:|:--|
| 72 | 86.118620 | 85.071354 | +1.216% | [1 2 4 5] |
| 73 | 85.408155 | 83.601644 | +2.115% | [1 2 3 4 5] |
| 74 | 86.384056 | 82.011795 | +5.061% | [1 2 3 4 5] |
| 75 | 85.605271 | 80.345140 | +6.145% | [1 2 3 4 5 6] |
| 76 | 82.342302 | 77.414851 | +5.984% | [1 2 3 4 5 6] |
| 77 | 81.628263 | 74.450268 | +8.794% | [1 2 3 4 5 6] |
| 78 | 83.556650 | 76.850507 | +8.026% | [1 2 3 4 5 6] |
| 79 | 81.253892 | 74.198416 | +8.683% | [1 2 3 4 5 6] |

- Formation gains: `[2.452 4.107 7.711 8.97 11.25 0.1254]%`
- Minimum after maturity: `+5.984%`
- F6 non-gateway terminal gain: `+0.000%`
- Worst sensor / minimum formation: `+14.717% / +0.125%`
- Minimum formation-time gain: `+0.000%`
- Window / terminal consensus: `+9.205% / +16.183%`
- Static / candidate runtime: `251.51 / 245.03 s`
- Auxiliary-state maintenance cost included: `0`
- Registered gate passed: `1`

## Evidence boundary

V126 is a privileged paired X36 seed-211 t=72 H=8 state-recovery upper bound. It retains the complete V105 static carrier, fusion weights and protection schedule. On only the formation-page cells whose opened V105 gain is negative, the post-fusion posterior is replaced before extraction by the same node posterior captured from the paired static full-payload arm. The intervention therefore tests whether an independently maintained safe shadow state can preserve V105 aggregate headroom while eliminating accumulated local regret. The rollback cells and counterfactual shadow states use opened alternative-arm outcomes, so V126 is not deployable and cannot support validation or generalization claims. Measurements, delivery uniforms, filter RNG, topology, weights and candidate communication accounting remain paired.
