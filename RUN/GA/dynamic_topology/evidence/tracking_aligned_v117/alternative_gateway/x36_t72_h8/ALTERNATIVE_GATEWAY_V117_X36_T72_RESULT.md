# V117 same-source alternative F6 gateway: X36 t72 H=8

- V113/V114/V116 endpoints reused: `1 / 1 / 1`
- V117 candidate screen reused: `0`
- Source / original gateway: `27 -> 32`
- Any candidate passed: `0`
- Oracle action: `v117-entry27-to33-return34-to2-h8`

| Arm | Mean E-OSPA | Gain vs CW | vs V113 | vs V114 | vs V116-5 | Mature min | Min form. | F6 peers | Worst | Bytes | Gate |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|:--:|
| CCW full | 84.037151 | -- | -- | -- | -- | -- | -- | -- | -- | -- | 1 |
| CW full | 81.803484 | -- | -- | -- | -- | -- | -- | -- | -- | -- | 1 |
| V113 gateway-32 full | 78.479689 | -- | -- | -- | -- | -- | -- | -- | -- | -- | 1 |
| V114 gateway-32 empty | 78.319230 | -- | -- | -- | -- | -- | -- | -- | -- | -- | 1 |
| V116 gateway-32 top-5 | 78.401889 | -- | -- | -- | -- | -- | -- | -- | -- | -- | 1 |
| entry-33 / return-34 | 78.893034 | +3.558% | -0.527% | -0.733% | -0.626% | +2.037% | -4.331% | -1.976% | +3.082% | +3.361% | 0 |
| entry-35 / return-36 | 79.185942 | +3.200% | -0.900% | -1.107% | -1.000% | +1.362% | -5.501% | -10.048% | +1.083% | +3.241% | 0 |
| entry-34 / return-35 | 78.944955 | +3.494% | -0.593% | -0.799% | -0.693% | +2.916% | -4.329% | -10.046% | +6.164% | +3.400% | 0 |

## Oracle diagnostics

- Entry / return / action: `33 / 34 / v117-entry27-to33-return34-to2-h8`
- Formation gains: `[-0.2216 0.9785 10.62 4.576 9.913 -4.331]%`
- Per-page gains: `[0.8576 3.635 4.22 2.947 2.037 4.807 5.158 5.008]%`
- Gateway terminal gain: `-17.936%`
- Window / terminal consensus: `+9.494% / +11.802%`
- Message / row / weight / strong invariants: `1 / 1 / 1 / 1`
- Gate passed: `0`

## Evidence boundary

V117 is a privileged opened-development X36 seed-211 t=72 H=8 gateway-location oracle. It keeps the clockwise formation cycle, the V113 F2--F5 abstention schedule, message count and fusion-weight multiset fixed. The original 27-to-32 F5-to-F6 residual is moved to receiver 33, 35 or 34; the displaced internal F6 source simultaneously takes over the original 33-to-2 F6-to-F1 return residual. This paired entry-return move preserves every sensor's outward influence path and both formation-cycle edges. These receivers were frozen from opened current tracking errors; future H=8 outcomes choose only the reported oracle arm. V117 is not deployable, validation or generalization evidence.
