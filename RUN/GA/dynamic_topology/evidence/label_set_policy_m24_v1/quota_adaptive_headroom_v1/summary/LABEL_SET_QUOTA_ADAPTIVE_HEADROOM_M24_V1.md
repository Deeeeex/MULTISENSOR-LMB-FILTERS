# M24 quota-adaptive H=3 headroom audit

- Mean old / quota / persistent-rank / temporal oracle E-OSPA: `14.506045 / 14.393734 / 14.319245 / 14.190508`
- Gain vs old two-option oracle: `0.774%`
- Incremental vs persistent-rank oracle: `-0.520%`
- Incremental vs temporal oracle: `-1.432%` (gate `2.000%`)
- Refreshed old-oracle states: `3 / 4`
- Registered hard failures: `6` (Q10 states `2`, Q12 states `4`)
- Completed-action safety / all-actions feasible / headroom gates: `1 / 0 / 0`

| Seed | Source | Old oracle | Quota oracle | Best | Incremental | Completed | Failed | Safe |
|--:|:--|--:|--:|:--|--:|--:|--:|--:|
| 11 | learned | 15.371985 | 14.944265 | `Q3-Q3-Q3` | +2.782% | 6 | 2 | 1 |
| 19 | ccw | 19.856173 | 19.850353 | `Q3-Q3-Q3` | +0.029% | 6 | 2 | 1 |
| 23 | ccw | 5.901460 | 5.901460 | `LLL` | +0.000% | 7 | 1 | 1 |
| 27 | learned | 16.894561 | 16.878856 | `Q6-Q6-Q6` | +0.093% | 7 | 1 | 1 |

## Decision

The quota-only action family is `rejected`. The next opened training-only method direction is `joint-state-adaptive-cross-quota-and-residual-weight-with-a-physical-capacity-mask`.

## Boundary

This opened-training audit evaluates a persistent H=3 offline oracle over fixed residual cross-edge quotas on four adversarial M24 behavior states. Failed actions retain infinite risk. It does not establish a deployable selector, development, held-out M24 or X36 result.
