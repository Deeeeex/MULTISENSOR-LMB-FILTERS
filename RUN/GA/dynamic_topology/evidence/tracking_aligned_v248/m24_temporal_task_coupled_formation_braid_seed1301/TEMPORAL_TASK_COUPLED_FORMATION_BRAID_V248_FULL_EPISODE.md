# V248 temporal task-coupled routing comparison

- Preset: `m24-formation-fov-temporal-coupled-formation-braid`
- Seed: `1301`
- Generation commit: `78735d23f8c85d350ad80e5ddb6d44bb2736d310`
- Realised temporal scene gate passed: `1`
- Balanced direction passed: `1`
- Paper threshold passed: `0`
- Next method decision: `retain-minimum-backbone-and-test-posterior-valued-residual-edges-on-the-corrected-scene`

| Arm | Full E-OSPA | Full RMSE | Focus consistency | Attempted bytes | Messages / step |
|:--|--:|--:|--:|--:|--:|
| Fixed formation tree | 125.478 | 22.640 | 133.599 | 40769168 | 46--48 |
| Full causal repair | 122.380 | 14.081 | 131.913 | 44867136 | 48--48 |
| Minimum causal backbone | 122.462 | 12.183 | 131.664 | 36675624 | 30--30 |

| Candidate over fixed | E-OSPA | RMSE | Focus consistency | Attempted-byte saving | Weakest formation E / RMSE |
|:--|--:|--:|--:|--:|--:|
| Full causal repair | +2.469% | +37.805% | +1.262% | -10.052% | +1.515% / -6.053% |
| Minimum causal backbone | +2.403% | +46.190% | +1.449% | +10.041% | +0.272% / -24.085% |

## Evidence boundary

V248 is a three-arm paired result on one opened M24 seed in the V247 temporally task-coupled scene. The scene gate uses realised visibility ownership and no tracking outcome. The result can compare fixed routing, full causal repair and the minimum backbone on this executed case, but it does not establish held-out, cross-scale or paper-level benefit.
