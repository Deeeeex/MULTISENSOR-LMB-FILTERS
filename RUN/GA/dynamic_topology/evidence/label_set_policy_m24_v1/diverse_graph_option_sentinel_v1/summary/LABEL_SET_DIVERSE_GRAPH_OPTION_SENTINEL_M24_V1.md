# M24 diverse persistent-rank H=3 sentinel

- States: `4`
- Mean CCC / old oracle / diverse oracle E-OSPA: `15.013510 / 14.506045 / 14.319245`
- Incremental oracle gain: `1.288%` (gate `2.000%`)
- Refreshed old oracle states: `3 / 4`
- States with >=5% gain versus CCC: `2 / 4`
- Hard audit / sentinel headroom gates: `1 / 0`

| Seed | Source | CCC | G1 | Old oracle | Diverse oracle | Best | Incremental | Gain vs CCC | Safe |
|--:|:--|--:|--:|--:|--:|:--|--:|--:|--:|
| 11 | learned | 15.371985 | 17.394177 | 15.371985 | 15.371985 | `CCC` | +0.000% | +0.000% | 1 |
| 19 | ccw | 19.856173 | 20.285314 | 19.856173 | 19.440435 | `G5-G5-G5` | +2.094% | +2.094% | 1 |
| 23 | ccw | 7.341908 | 5.901460 | 5.901460 | 5.896317 | `G5-G5-G5` | +0.087% | +19.690% | 1 |
| 27 | learned | 17.483976 | 16.894561 | 16.894561 | 16.568242 | `G4-G4-G4` | +1.932% | +5.238% | 1 |

## Decision

The persistent-rank proposal bank is `rejected`. The next opened training-only method direction is `causal-time-structured-h3-options-over-safe-diverse-ranks`.

## Boundary

This opened-training sentinel measures offline oracle headroom of six persistent safe GNN graph ranks on four adversarial M24 behavior states. It does not establish a selector, development, held-out M24 or X36 result.
