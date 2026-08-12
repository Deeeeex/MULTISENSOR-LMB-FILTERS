# V127 local-posterior rollback: X36 t72 H=8

- Matched baseline: `matched-static-fixed-ccw-full-payload`
- Frozen H=8 reference reused: `1`

| Arm | Mean E-OSPA | Gain vs static | Bytes saving |
|:--|--:|--:|--:|
| Static full payload | 84.037151 | -- | -- |
| V127 local-posterior rollback | 79.528581 | +5.365% | +6.085% |

| t | Static | Candidate | Gain | Protected formations |
|--:|--:|--:|--:|:--|
| 72 | 86.118620 | 85.071354 | +1.216% | [1 2 4 5] |
| 73 | 85.408155 | 83.601644 | +2.115% | [1 2 3 4 5] |
| 74 | 86.384056 | 82.011795 | +5.061% | [1 2 3 4 5] |
| 75 | 85.605271 | 80.345140 | +6.145% | [1 2 3 4 5 6] |
| 76 | 82.342302 | 77.510170 | +5.868% | [1 2 3 4 5 6] |
| 77 | 81.628263 | 74.449569 | +8.794% | [1 2 3 4 5 6] |
| 78 | 83.556650 | 77.613864 | +7.112% | [1 2 3 4 5 6] |
| 79 | 81.253892 | 75.625109 | +6.927% | [1 2 3 4 5 6] |

- Formation gains: `[-0.9541 5.012 7.711 8.97 11.25 0.407]%`
- Minimum after maturity: `+5.868%`
- F6 non-gateway terminal gain: `-1.471%`
- Worst sensor / minimum formation: `+16.701% / -0.954%`
- Minimum formation-time gain: `-15.874%`
- Window / terminal consensus: `+9.332% / +15.740%`
- Static / candidate runtime: `251.51 / 256.88 s`
- Auxiliary-state maintenance cost included: `1`
- Registered gate passed: `0`

## Evidence boundary

V127 is a paired X36 seed-211 t=72 H=8 state-source attribution. It retains the complete V105 route, fusion weights and protection schedule, and retains the opened V126 rollback node-time mask. At those cells it replaces the post-fusion working posterior with the same receiver node's current measurement-updated local posterior. That state is already computed by the standard filter and requires no additional inter-node payload or parallel shadow filter. The rollback timing still comes from opened outcomes, so V127 tests state source feasibility only and is not an online policy, validation, or generalization claim. Measurements, delivery uniforms, filter RNG, topology, weights and communication accounting remain paired.
