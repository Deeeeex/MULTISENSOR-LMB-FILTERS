# V116 privileged delayed label-return oracle: X36 t72 H=8

- V113 baselines reused: `1`
- V114 empty boundary reused: `1`
- V116 candidate screen reused: `0`
- Boundary edge: `27 -> 32`
- Any candidate passed: `0`
- Oracle action: `v116-truth-ranked-top5-labels-p5-h8`

| Arm | Mean E-OSPA | Gain vs CW | vs V113 | vs V114 | Mature min | Min form. | F6 peers | Worst | Bytes | Gate |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|:--:|
| CCW full | 84.037151 | -- | -- | -- | -- | -- | -- | -- | -- | 1 |
| CW full | 81.803484 | -- | -- | -- | -- | -- | -- | -- | -- | 1 |
| V113 full boundary | 78.479689 | -- | -- | -- | -- | -- | -- | -- | -- | 1 |
| V114 empty boundary | 78.319230 | -- | -- | -- | -- | -- | -- | -- | -- | 1 |
| top-5 | 78.401889 | +4.158% | +0.099% | -0.106% | +2.769% | -0.921% | -5.594% | +8.965% | +4.030% | 0 |
| top-10 | 78.462017 | +4.085% | +0.023% | -0.182% | +2.769% | -1.365% | -7.240% | +7.513% | +3.802% | 0 |
| top-15 | 78.442341 | +4.109% | +0.048% | -0.157% | +2.960% | -1.219% | -7.209% | +8.219% | +3.570% | 0 |

## Oracle diagnostics

- Quota / action: `5 / v116-truth-ranked-top5-labels-p5-h8`
- Selected labels by page: `[0 0 0 0 5 5 5 5]`
- Candidate labels by page: `[70 92 91 91 114 114 114 115]`
- Formation gains: `[-6.874e-05 0.9785 10.62 4.576 9.913 -0.9207]%`
- Per-page gains: `[0.9093 3.021 4.154 2.96 2.769 7.728 6.148 5.981]%`
- Boundary receiver terminal gain: `-0.036%`
- Window / terminal consensus: `+12.135% / +17.974%`
- Gate passed: `0`

## Evidence boundary

V116 is a privileged opened-development X36 seed-211 t=72 H=8 action-space oracle. It keeps the V113 clockwise carrier and F2--F5 whole-source abstention fixed. On pages five through eight, sender-27 labels are matched one-to-one to current target states using privileged truth and ranked by Euclidean position error. The top 5, 10 or 15 complete labels are frozen before candidate outcomes are opened; omitted labels explicitly abstain. Future H=8 tracking outcomes choose only the reported oracle arm. V116 is not deployable, validation or generalization evidence.
