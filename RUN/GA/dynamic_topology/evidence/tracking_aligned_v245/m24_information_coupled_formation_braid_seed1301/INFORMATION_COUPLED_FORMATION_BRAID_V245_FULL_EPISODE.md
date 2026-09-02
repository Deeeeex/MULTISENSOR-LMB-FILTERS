# V245 coupled formation-braid routing comparison

- Preset: `m24-formation-fov-coupled-formation-braid`
- Seed: `1301`
- Generation commit: `c5db1c1351f3c90908c03d65875d34c98771098a`
- Balanced direction passed: `0`
- Paper threshold passed: `0`
- Next method decision: `minimum-backbone-removed-useful-information-add-selective-residual-edges`

| Arm | Full E-OSPA | Full RMSE | Focus consistency | Attempted bytes | Messages / step |
|:--|--:|--:|--:|--:|--:|
| Fixed formation tree | 126.724 | 9.052 | 135.624 | 34363568 | 46--48 |
| Full causal repair | 125.394 | 8.947 | 133.109 | 38421672 | 48--48 |
| Minimum causal backbone | 125.991 | 9.579 | 132.815 | 26465520 | 30--30 |

| Candidate over fixed | E-OSPA | RMSE | Focus consistency | Attempted-byte saving | Weakest formation E / RMSE |
|:--|--:|--:|--:|--:|--:|
| Full causal repair | +1.050% | +1.161% | +1.854% | -11.809% | -0.088% / -0.498% |
| Minimum causal backbone | +0.578% | -5.826% | +2.071% | +22.984% | -0.191% / -9.886% |

## Evidence boundary

V245 is a three-arm paired result on one opened M24 seed. It can attribute the effect of aligning target handoffs with failed formation-tree cuts and choose the next development method, but it does not establish held-out, cross-scale, or paper-level benefit.
