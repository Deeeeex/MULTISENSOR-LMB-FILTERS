# V122 upstream F5 release: X36 t72 H=8

- V113 baselines reused: `1`
- V122 candidate screen reused: `0`
- Gate passed: `0`

| Arm | Mean E-OSPA | Gain vs CW | vs V113 | Mature min | Min form. | Terminal form. | F5 | F6 | F6 peers | Worst | Bytes | Gate |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|:--:|
| V122 release F5 | 79.585550 | +2.711% | -1.409% | +0.640% | -0.014% | -0.125% | -0.014% | -0.000% | +0.000% | +1.770% | +1.710% | 0 |

## Diagnostics

- Formation gains: `[0 0.9785 10.62 4.576 -0.01405 -2.167e-08]%`
- Terminal formation gains: `[0 4.858 16.38 10.84 -0.1252 -1.927e-07]%`
- Per-page gains: `[0 1.394 2.536 1.838 0.6403 4.826 5.202 5.627]%`
- F6 peer repair vs V113: `+6.722%`
- Window / terminal consensus: `+6.762% / +12.232%`
- Rolling B3: `1`

## Evidence boundary

V122 is a single-arm privileged X36 seed-211 t=72 H=8 causal screen. It keeps the fixed clockwise carrier and the V113 explicit-abstention mechanism, but removes F5 from the protected set on every page while retaining F2--F4. This tests whether the delayed F6 loss is propagated by an altered F5 posterior rather than caused by F6 input geometry. V113 CCW full, CW full and F2--F5 abstention outcomes are reused. Measurements, delivery uniforms, filter RNG, topology, weights and communication accounting remain paired. The action was chosen after V121 opened outcomes and is development evidence only, not validation or generalization evidence.
