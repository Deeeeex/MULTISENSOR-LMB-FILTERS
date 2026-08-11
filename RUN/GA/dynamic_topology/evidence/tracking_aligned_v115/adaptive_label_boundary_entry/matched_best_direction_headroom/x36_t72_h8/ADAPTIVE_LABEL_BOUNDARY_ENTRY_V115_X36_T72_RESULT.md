# V115 adaptive label-wise F6 boundary entry: X36 t72 H=8

- V113 baselines reused: `1`
- V114 early shield reused: `1`
- V115 candidate screen reused: `0`
- Boundary edge: `27 -> 32`
- Any candidate passed: `0`
- Oracle action: `v115-boundary-supported-labels-p5-h8`

| Arm | Mean E-OSPA | Gain vs CW | vs V113 | vs V114 | Mature min | Min form. | F6 peers | Worst | Bytes | Gate |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|:--:|
| CCW full | 84.037151 | -- | -- | -- | -- | -- | -- | -- | -- | 1 |
| CW full | 81.803484 | -- | -- | -- | -- | -- | -- | -- | -- | 1 |
| V113 unshielded | 78.479689 | -- | -- | -- | -- | -- | -- | -- | -- | 1 |
| V114 whole shield | 78.319230 | -- | -- | -- | -- | -- | -- | -- | -- | 1 |
| `v115-boundary-supported-labels-p5-h8` | 78.479671 | +4.063% | +0.000% | -0.205% | +2.588% | -1.495% | -7.206% | +6.851% | +3.419% | 0 |
| `v115-boundary-supported-or-high-r-p5-h8` | 78.479683 | +4.063% | +0.000% | -0.205% | +2.588% | -1.495% | -7.206% | +6.850% | +3.373% | 0 |
| `v115-boundary-receiver-need-p5-h8` | 78.479672 | +4.063% | +0.000% | -0.205% | +2.588% | -1.495% | -7.206% | +6.851% | +3.359% | 0 |

## Oracle diagnostics

- Payload mode: `boundary-label-sender-supported-only`
- Selected labels by page: `[0 0 0 0 19 16 18 19]`
- Candidate labels by page: `[70 92 91 91 114 114 114 115]`
- Formation gains: `[-6.874e-05 0.9785 10.62 4.576 9.913 -1.495]%`
- Per-page gains: `[0.9093 3.021 4.154 2.96 2.588 7.534 5.941 5.781]%`
- Boundary receiver terminal gain: `+0.029%`
- Window / terminal consensus: `+11.670% / +17.188%`
- Gate passed: `0`

## Evidence boundary

V115 is an opened-development X36 seed-211 t=72 H=8 label-action screen. It keeps the V113 clockwise carrier and F2--F5 whole-source abstention fixed. Starting on page five, only the F6 boundary edge 27-to-32 changes from whole-posterior shielding to complete-label whitelisting. The three rules use current sender association support, sender existence, or current receiver need; omitted labels explicitly abstain and are not interpreted as negative evidence. Truth and future outcomes do not select labels at runtime. The action family and anchor were chosen after V114, so this is method-development evidence, not validation or cross-scene generalization.
