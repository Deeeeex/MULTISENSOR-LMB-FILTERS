# V114 F6 influence-boundary shield: X36 t72 H=8

- V113 baselines reused: `1`
- V114 candidate screen reused: `1`
- Boundary edge: `27 -> 32`
- Any candidate passed: `0`
- Oracle action: `v114-f6-shield-early-p5-h8`

| Arm | Shield start | Mean E-OSPA | Gain vs CW full | Marginal vs V113 | Mature min | Min formation | F6 peers | F6 repair vs V113 | Bytes | Gate |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|:--:|
| CCW full | -- | 84.037151 | -- | -- | -- | -- | -- | -- | -- | 1 |
| CW full | -- | 81.803484 | -- | -- | -- | -- | -- | -- | -- | 1 |
| V113 unshielded | -- | 78.479689 | +4.063% | -- | +2.588% | -1.495% | -7.206% | -- | +3.330% | 1 |
| `v114-f6-shield-on-arrival-p6-h8` | 6 | 78.358108 | +4.212% | +0.155% | +2.588% | -0.598% | -5.604% | +1.494% | +4.029% | 0 |
| `v114-f6-shield-early-p5-h8` | 5 | 78.319230 | +4.259% | +0.204% | +2.960% | -0.311% | -5.522% | +1.571% | +4.244% | 0 |

## Oracle diagnostics

- Formation gains vs CW full: `[-6.874e-05 0.9785 10.62 4.576 9.913 -0.3105]%`
- Per-page gains vs CW full: `[0.9093 3.021 4.154 2.96 2.962 7.938 6.357 6.201]%`
- Boundary receiver terminal gain: `+7.329%`
- Worst-sensor gain: `+11.136%`
- Window / terminal consensus: `+12.671% / +19.586%`
- Gate passed: `0`

## Evidence boundary

V114 is a frozen retrospective X36 seed-211 t=72 H=8 boundary-control screen. V113 opened outcomes identify the clockwise 27-to-32 cross-formation residual edge and the sixth page as the first F6 loss page. V114 therefore holds the V113 clockwise F2--F5 abstention mechanism fixed and adds F6 receiver-side abstention either on the predicted arrival page or one page earlier. The V113 CCW-full, CW-full and unshielded mechanism outcomes are reused; only the two boundary arms are executed. Measurements, delivery uniforms, filter RNG, carrier weights, physical edges and communication accounting remain paired, and rolling B3 is preserved. Because the boundary and timing were selected from opened V113 outcomes, V114 is an oracle action-space screen, not a deployable policy or validation claim.
