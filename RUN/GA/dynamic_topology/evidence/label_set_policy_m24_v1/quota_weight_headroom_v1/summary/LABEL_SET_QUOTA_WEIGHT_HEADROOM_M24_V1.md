# M24 quota-weight H=3 headroom audit

- Mean old / larger-weight / Q-e05 / temporal / combined oracle E-OSPA: `14.506045 / 14.506045 / 14.393734 / 14.190508 / 14.190508`
- Incremental vs temporal oracle: `+0.000%` (gate `2.000%`)
- New-weight refresh old / temporal states: `0 / 0`
- Registered hard failures: `0`
- Hard audit / headroom gates: `1 / 0`

| Seed | Source | Old | Q-e05 | Temporal | Best new weight | New-weight E-OSPA | Gain vs old | Safe |
|--:|:--|--:|--:|--:|:--|--:|--:|--:|
| 11 | learned | 15.371985 | 14.944265 | 14.897434 | `Q3E10-Q3E10-Q3E10` | 16.509132 | -7.398% | 1 |
| 19 | ccw | 19.856173 | 19.850353 | 19.440435 | `Q3E10-Q3E10-Q3E10` | 20.858002 | -5.045% | 1 |
| 23 | ccw | 5.901460 | 5.901460 | 5.896317 | `Q3E10-Q3E10-Q3E10` | 6.793761 | -15.120% | 1 |
| 27 | learned | 16.894561 | 16.878856 | 16.527847 | `Q6E10-Q6E10-Q6E10` | 16.917613 | -0.136% | 1 |

## Decision

The larger residual-weight family is `rejected`. The next opened training-only method direction is `sub-e05-or-nodewise-risk-calibrated-residual-weighting-before-any-selector-fit`.

## Boundary

This opened-training audit evaluates persistent H=3 residual weights 0.10, 0.20 and 0.25 at safe Q3/Q6 on four adversarial M24 behavior states. It does not establish a deployable selector, development, held-out M24 or X36 result.
