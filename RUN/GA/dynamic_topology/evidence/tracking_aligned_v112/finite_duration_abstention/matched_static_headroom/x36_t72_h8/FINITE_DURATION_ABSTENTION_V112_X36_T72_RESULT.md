# V112 finite-duration abstention labels: X36 t72 H=8

- Matched baseline: `matched-static-fixed-ccw-full-payload`
- Frozen H=8 reference reused: `1`
- Oracle duration: `5` step(s)
- Any candidate passed: `0`

| Duration | Mean E-OSPA | Gain | Min post-maturity | Min formation | Downstream regret | F6 peers | Byte saving | Window cons. | Terminal cons. | Gate |
|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|:--:|
| 3 | 81.765760 | +2.703% | +0.092% | -0.928% | +0.928% | -5.743% | +1.103% | +1.000% | -2.509% | 0 |
| 4 | 81.466081 | +3.059% | +1.901% | -0.865% | +0.865% | -5.752% | +2.036% | +3.267% | +1.041% | 0 |
| 5 | 80.737392 | +3.927% | +2.951% | -0.865% | +0.865% | -5.752% | +2.729% | +5.216% | +2.194% | 0 |
| 6 | 80.423632 | +4.300% | +1.957% | -0.931% | +0.931% | -2.947% | +3.495% | +7.131% | +5.127% | 0 |

## Oracle label

- Action: `v112-abstain-5-then-full-h8`
- Formation gains: `[-0.8652 1.762 5.927 7.136 10.55 -0.551]%`
- Per-page gains: `[1.216 2.115 5.061 6.145 5.188 4.137 2.951 4.673]%`
- Gate passed: `0`

## Evidence boundary

V112 is a frozen retrospective H=8 action-family label screen. Each candidate follows the V105 formation schedule with explicit source abstention for exactly 3, 4, 5 or 6 pages and then restores the matched static full-payload route and fusion inputs for all remaining pages. Measurements, delivery uniforms, filter RNG, communication accounting and the frozen reference outcome remain paired. Truth and future outcomes are used only to label mean gain and worst downstream formation regret after execution. V112 is an action-space headroom study, not a deployable policy or a validation/generalization claim.
