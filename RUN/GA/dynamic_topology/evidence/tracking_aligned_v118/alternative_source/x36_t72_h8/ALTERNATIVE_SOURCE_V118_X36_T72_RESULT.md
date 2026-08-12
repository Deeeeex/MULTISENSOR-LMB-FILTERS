# V118 fixed-entry alternative F5 source: X36 t72 H=8

- V113/V114/V116 endpoints reused: `1 / 1 / 1`
- V118 candidate screen reused: `0`
- Original source / receiver: `27 -> 32`
- Any candidate passed: `0`
- Oracle action: `v118-source30-to32-h8`

| Arm | Mean E-OSPA | Gain vs CW | vs V113 | vs V114 | vs V116-5 | Mature min | Min form. | F6 peers | Worst | Bytes | Gate |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|:--:|
| CCW full | 84.037151 | -- | -- | -- | -- | -- | -- | -- | -- | -- | 1 |
| CW full | 81.803484 | -- | -- | -- | -- | -- | -- | -- | -- | -- | 1 |
| V113 source-27 full | 78.479689 | -- | -- | -- | -- | -- | -- | -- | -- | -- | 1 |
| V114 source-27 empty | 78.319230 | -- | -- | -- | -- | -- | -- | -- | -- | -- | 1 |
| V116 source-27 top-5 | 78.401889 | -- | -- | -- | -- | -- | -- | -- | -- | -- | 1 |
| source-25 | 78.609175 | +3.905% | -0.165% | -0.370% | -0.264% | +3.038% | -2.326% | -7.412% | +6.089% | +3.213% | 0 |
| source-26 | 78.598787 | +3.918% | -0.152% | -0.357% | -0.251% | +2.677% | -2.232% | -7.234% | +6.601% | +3.034% | 0 |
| source-28 | 78.426149 | +4.129% | +0.068% | -0.137% | -0.031% | +3.418% | -0.991% | -7.305% | +9.616% | +3.304% | 0 |
| source-29 | 78.512258 | +4.023% | -0.041% | -0.246% | -0.141% | +2.838% | -1.761% | -7.260% | +8.083% | +3.175% | 0 |
| source-30 | 78.525158 | +4.008% | -0.058% | -0.263% | -0.157% | +2.675% | -2.130% | -0.416% | +6.237% | +2.908% | 0 |

## Oracle diagnostics

- Source / receiver / action: `30 / 32 / v118-source30-to32-h8`
- Formation gains: `[0.2823 0.9785 10.62 4.576 9.939 -2.13]%`
- Per-page gains: `[0.6942 2.563 4.361 3.004 2.675 6.293 6.3 6.589]%`
- Receiver terminal gain: `-0.177%`
- Window / terminal consensus: `+10.988% / +17.217%`
- Message / row / weight / strong invariants: `1 / 1 / 1 / 1`
- Gate passed: `0`

## Evidence boundary

V118 is a privileged opened-development X36 seed-211 t=72 H=8 single-source oracle. It keeps receiver 32, the clockwise formation carrier, the V113 F2--F5 abstention schedule, the F6 internal and return paths, message count and fusion weights fixed. The original 27-to-32 residual is replaced by each other F5 sender, 25, 26, 28, 29 or 30. Because that cross edge is node 27's only outward influence, every candidate also replaces the fixed 25-to-26 intra-F5 edge by 27-to-26. This minimum paired role migration preserves connectivity without adding a message. All five alternatives are exhausted; future H=8 outcomes choose only the reported oracle arm. V118 is not deployable, validation or generalization evidence.
